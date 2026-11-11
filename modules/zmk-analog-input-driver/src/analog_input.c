/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_analog_input

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/input/input.h>
#include <stdlib.h> //for abs()
#include <zephyr/sys/util.h> // for CLAMP
#include <string.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ANALOG_INPUT, CONFIG_ANALOG_INPUT_LOG_LEVEL);

#include <zmk/drivers/analog_input.h>

#if IS_ENABLED(CONFIG_SETTINGS)
#include <zephyr/settings/settings.h>
#endif

#if IS_ENABLED(CONFIG_SHELL)
#include <zephyr/shell/shell.h>
#endif

static const struct device *stick_dev;

#if IS_ENABLED(CONFIG_SETTINGS)
/* Single blob — fewer flash writes (RP2040 NVS is sensitive). */
struct stick_calib_store {
	uint16_t mid_x;
	uint16_t mid_y;
	uint16_t divisor;
	uint8_t deadzone;
	uint8_t invert_y;
	uint8_t valid;
	uint8_t _pad[2];
};

static struct stick_calib_store stick_store;

static int stick_settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg) {
	const char *next;
	ssize_t rc;

	if (settings_name_steq(name, "calib", &next) && !next) {
		if (len != sizeof(stick_store)) {
			return -EINVAL;
		}
		rc = read_cb(cb_arg, &stick_store, len);
		return rc >= 0 ? 0 : rc;
	}
	return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(psp_stick, "psp_stick", NULL, stick_settings_set, NULL, NULL);

static void stick_apply_store(struct analog_input_data *data) {
	if (!stick_store.valid || data->rt_ch_len < 2) {
		return;
	}
	data->rt_ch[0].mv_mid = stick_store.mid_x;
	data->rt_ch[1].mv_mid = stick_store.mid_y;
	data->rt_ch[0].mv_deadzone = stick_store.deadzone;
	data->rt_ch[1].mv_deadzone = stick_store.deadzone;
	data->rt_ch[0].scale_divisor = stick_store.divisor ? stick_store.divisor : 1;
	data->rt_ch[1].scale_divisor = stick_store.divisor ? stick_store.divisor : 1;
	data->rt_ch[1].invert = stick_store.invert_y != 0;
}

static int stick_save_store(struct analog_input_data *data) {
	/* Copy to stack so key/value are in RAM while flash XIP is off */
	struct stick_calib_store tmp;

	tmp.mid_x = data->rt_ch[0].mv_mid;
	tmp.mid_y = data->rt_ch[1].mv_mid;
	tmp.deadzone = data->rt_ch[0].mv_deadzone;
	tmp.divisor = data->rt_ch[0].scale_divisor;
	tmp.invert_y = data->rt_ch[1].invert ? 1 : 0;
	tmp.valid = 1;
	tmp._pad[0] = 0;
	tmp._pad[1] = 0;
	stick_store = tmp;
	return settings_save_one("psp_stick/calib", &tmp, sizeof(tmp));
}
#endif /* CONFIG_SETTINGS */

static int analog_input_report_data(const struct device *dev) {
    struct analog_input_data *data = dev->data;

    if (unlikely(!data->ready)) {
        LOG_WRN("Device is not initialized yet");
        return -EBUSY;
    }

#if CONFIG_ANALOG_INPUT_REPORT_INTERVAL_MIN > 0
    static int64_t last_smp_time = 0;
    static int64_t last_rpt_time = 0;
    int64_t now = k_uptime_get();
#endif

    struct adc_sequence* as = &data->as;

    for (uint8_t i = 0; i < data->rt_ch_len; i++) {
        struct analog_input_io_channel ch_cfg = data->rt_ch[i];
        const struct device* adc = ch_cfg.adc_channel.dev;

        if (i == 0) {
#ifdef CONFIG_ADC_ASYNC
            int err = adc_read_async(adc, as, &data->async_sig);
            if (err < 0) {
                LOG_ERR("AIN%u read_async returned %d", i, err);
                return err;
            }
            err = k_poll(&data->async_evt, 1, K_FOREVER);
            if (err < 0) {
                LOG_ERR("AIN%u k_poll returned %d", i, err);
                return err;
            }
            if (!data->async_evt.signal->signaled) {
                return 0;
            }
            data->async_evt.signal->signaled = 0;
    	    data->async_evt.state = K_POLL_STATE_NOT_READY;
#else
            int err = adc_read(adc, as);
            if (err < 0) {
                LOG_ERR("AIN%u read returned %d", i, err);
                return err;
            }
#endif
        }

        int32_t raw = data->as_buff[i];
        int32_t mv = raw;
        adc_raw_to_millivolts(adc_ref_internal(adc), ADC_GAIN_1_6, as->resolution, &mv);
        data->last_raw[i] = raw;
        data->last_mv[i] = mv;

        int16_t v = mv - ch_cfg.mv_mid;
        int16_t dz = ch_cfg.mv_deadzone;
        if (dz) {
            if (v > 0) {
                if (v < dz) v = 0; else v -= dz;
            }
            if (v < 0) {
                if (v > -dz) v = 0; else v += dz;
            }
        }
        uint16_t mm = ch_cfg.mv_min_max;
        if (mm) {
            if (v > 0 && v > mm) v = mm;
            if (v < 0 && v < -mm) v = -mm;
        }

        if (ch_cfg.invert) v *= -1;
        v = (int16_t)((v * ch_cfg.scale_multiplier) / ch_cfg.scale_divisor);

        if (ch_cfg.report_on_change_only) {
            // track raw value to compare until next report interval
            data->delta[i] = v;
        }
        else {
            // accumulate delta until report in next iteration
            int32_t delta = data->delta[i];
            int32_t dv = delta + v;
            data->delta[i] = dv;
        }
    }

    // First read is setup as calibration
    as->calibrate = false;

#if IS_ENABLED(CONFIG_ANALOG_INPUT_LOG_DBG_RAW)
    /* ~10 Hz max — full-rate DBG floods USB CDC and shell commands time out */
    static int64_t last_raw_log;
    int64_t raw_now = k_uptime_get();
    if (raw_now - last_raw_log >= 100) {
        last_raw_log = raw_now;
        for (uint8_t i = 0; i < data->rt_ch_len; i++) {
            LOG_INF("AIN%u raw: %d mv: %d", data->rt_ch[i].adc_channel.channel_id,
                    data->last_raw[i], data->last_mv[i]);
        }
    }
#endif

#if CONFIG_ANALOG_INPUT_REPORT_INTERVAL_MIN > 0
    // purge accumulated delta, if last sampled had not been reported on last report tick
    if (now - last_smp_time >= CONFIG_ANALOG_INPUT_REPORT_INTERVAL_MIN) {
        for (uint8_t i = 0; i < data->rt_ch_len; i++) {
            data->delta[i] = 0;
            data->prev[i] = 0;
        }
    }
    last_smp_time = now;
#endif

#if CONFIG_ANALOG_INPUT_REPORT_INTERVAL_MIN > 0
    // strict to report inerval
    if (now - last_rpt_time < CONFIG_ANALOG_INPUT_REPORT_INTERVAL_MIN) {
        return 0;
    }
#endif

    if (!data->actived) {
        return 0;
    }

    int8_t idx_to_sync = -1;
    for (int8_t i = (int8_t)data->rt_ch_len - 1; i >= 0; i--) {
        int32_t dv = data->delta[i];
        int32_t pv = data->prev[i];
        if (dv != pv) {
            idx_to_sync = i;
            break;
        }
    }

    for (uint8_t i = 0; i < data->rt_ch_len; i++) {
        struct analog_input_io_channel ch_cfg = data->rt_ch[i];
        int32_t dv = data->delta[i];
        int32_t pv = data->prev[i];
        if (dv != pv) {
#if CONFIG_ANALOG_INPUT_REPORT_INTERVAL_MIN > 0
            last_rpt_time = now;
#endif
            data->delta[i] = 0;
            if (ch_cfg.report_on_change_only) {
                data->prev[i] = dv;
            }

#if IS_ENABLED(CONFIG_ANALOG_INPUT_LOG_DBG_REPORT)
            LOG_DBG("input_report %u rv: %d  e:%d  c:%d", i, dv, ch_cfg.evt_type, ch_cfg.input_code);
#endif
            input_report(dev, ch_cfg.evt_type, ch_cfg.input_code, dv, i == idx_to_sync, K_NO_WAIT);
        }
    }
    return 0;
}

K_THREAD_STACK_DEFINE(analog_input_q_stack, CONFIG_ANALOG_INPUT_WORKQUEUE_STACK_SIZE);

static struct k_work_q analog_input_work_q;

static void sampling_work_handler(struct k_work *work) {
    struct analog_input_data *data = CONTAINER_OF(work, struct analog_input_data, sampling_work);
    // LOG_DBG("sampling work triggered");
    analog_input_report_data(data->dev);
}

static void sampling_timer_handler(struct k_timer *timer) {
    struct analog_input_data *data = CONTAINER_OF(timer, struct analog_input_data, sampling_timer);
    // LOG_DBG("sampling timer triggered");
    k_work_submit_to_queue(&analog_input_work_q, &data->sampling_work);
    k_work_submit(&data->sampling_work);
}

static int active_set_value(const struct device *dev, bool active) {
    struct analog_input_data *data = dev->data;
    if (data->actived == active) return 0;
    LOG_DBG("%d", active ? 1 : 0);
    data->actived = active;
    return 0;
}

static int sample_hz_set_value(const struct device *dev, uint32_t hz) {
    struct analog_input_data *data = dev->data;

    if (unlikely(!data->ready)) {
        LOG_DBG("Device is not initialized yet");
        return -EBUSY;
    }

    if (data->enabled) {
        LOG_DBG("Device is busy, would not update sampleing rate in enable state.");
        return -EBUSY;
    }

    LOG_DBG("%d", hz);
    data->sampling_hz = hz;
    return 0;
}

static int enable_set_value(const struct device *dev, bool enable) {
    struct analog_input_data *data = dev->data;
    // const struct tb6612fng_config *config = dev->config;

    if (unlikely(!data->ready)) {
        LOG_DBG("Device is not initialized yet");
        return -EBUSY;
    }

    if (data->enabled == enable) {
        return 0;
    }
    
    LOG_DBG("%d", enable ? 1 : 0);
    if (enable) {
        if (data->sampling_hz != 0) {
            uint32_t usec = 1000000UL / data->sampling_hz;
            k_timer_start(&data->sampling_timer, K_USEC(usec), K_USEC(usec));
        } else {
            k_timer_start(&data->sampling_timer, K_NO_WAIT, K_NO_WAIT);
        }
        data->enabled = true;
    }
    else {
        k_timer_stop(&data->sampling_timer);
        data->enabled = false;
    }

    return 0;
}

static void analog_input_async_init(struct k_work *work) {
    struct k_work_delayable *work_delayable = (struct k_work_delayable *)work;
    struct analog_input_data *data = CONTAINER_OF(work_delayable, 
                                                  struct analog_input_data, init_work);
    const struct device *dev = data->dev;
    const struct analog_input_config *config = dev->config;

    uint32_t ch_mask = 0;

    data->rt_ch_len = config->io_channels_len;
    if (data->rt_ch_len > ANALOG_INPUT_MAX_CHANNELS) {
        data->rt_ch_len = ANALOG_INPUT_MAX_CHANNELS;
    }
    for (uint8_t i = 0; i < data->rt_ch_len; i++) {
        data->rt_ch[i] = config->io_channels[i];
    }

#if IS_ENABLED(CONFIG_SETTINGS)
    settings_load_subtree("psp_stick");
    stick_apply_store(data);
#endif

    for (uint8_t i = 0; i < data->rt_ch_len; i++) {
        struct analog_input_io_channel ch_cfg = data->rt_ch[i];
        const struct device* adc = ch_cfg.adc_channel.dev;
        uint8_t channel_id = ch_cfg.adc_channel.channel_id;
        
#if CONFIG_ANALOG_INPUT_USE_DTS_ADC_CH_CFG
        struct adc_channel_cfg channel_cfg = ch_cfg.adc_channel.channel_cfg;
#else
        struct adc_channel_cfg channel_cfg = {
            .gain = ADC_GAIN_1_6,
            .reference = ADC_REF_INTERNAL,
            .acquisition_time = ADC_ACQ_TIME_DEFAULT,
            .channel_id = channel_id,
            #ifdef CONFIG_ADC_CONFIGURABLE_INPUTS
                #ifdef CONFIG_ADC_NRFX_SAADC
                    .input_positive = SAADC_CH_PSELP_PSELP_AnalogInput0 + channel_id,
                #else /* CONFIG_ADC_NRFX_SAADC */
                    .input_positive = channel_id,
                #endif /* CONFIG_ADC_NRFX_SAADC */
            #endif /* CONFIG_ADC_CONFIGURABLE_INPUTS */
        };
#endif

        ch_mask |= BIT(channel_id);

        if (!device_is_ready(adc)) {
            LOG_ERR("AIN%u device is not ready %s", i, adc->name);
            continue;
        }

        int err = adc_channel_setup(adc, &channel_cfg);
        if (err < 0) {
            LOG_ERR("AIN%u setup returned %d", i, err);
        }
    }

    uint16_t delta_size = data->rt_ch_len * sizeof(int32_t);
    data->delta = malloc(delta_size);
    memset(data->delta, 0, delta_size);

    uint16_t prev_size = data->rt_ch_len * sizeof(int32_t);
    data->prev = malloc(prev_size);
    memset(data->prev, 0, prev_size);

#if CONFIG_ANALOG_INPUT_ADC_RES > 16
    uint16_t buff_size = data->rt_ch_len * sizeof(uint32_t);
#else
    uint16_t buff_size = data->rt_ch_len * sizeof(uint16_t);
#endif
    data->as_buff = malloc(buff_size);
    memset(data->as_buff, 0, buff_size);

    data->as = (struct adc_sequence){
        .channels = ch_mask,
        .buffer = data->as_buff,
        .buffer_size = buff_size,
        .oversampling = 0,
        .resolution = CONFIG_ANALOG_INPUT_ADC_RES,
        .calibrate = true,
    };

#ifdef CONFIG_ADC_ASYNC
    k_poll_signal_init(&data->async_sig);
    struct k_poll_event async_evt = K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
                                                             K_POLL_MODE_NOTIFY_ONLY,
                                                             &data->async_sig);
    data->async_evt = async_evt;
#endif

    data->ready = true;

    k_work_init(&data->sampling_work, sampling_work_handler);
    k_work_queue_start(&analog_input_work_q,
                        analog_input_q_stack, K_THREAD_STACK_SIZEOF(analog_input_q_stack),
                        CONFIG_ANALOG_INPUT_WORKQUEUE_PRIORITY, NULL);

    k_timer_init(&data->sampling_timer, sampling_timer_handler, NULL);

    sample_hz_set_value(dev, config->sampling_hz);
    active_set_value(dev, true);
    if (data->sampling_hz) {
        enable_set_value(dev, true);
    }

}

static int analog_input_init(const struct device *dev) {
    struct analog_input_data *data = dev->data;
    int err = 0;

    data->dev = dev;
    stick_dev = dev;
    k_work_init_delayable(&data->init_work, analog_input_async_init);
    k_work_schedule(&data->init_work, K_MSEC(1));

    return err;
}

static int analog_input_attr_set(const struct device *dev, enum sensor_channel chan,
                            enum sensor_attribute attr, const struct sensor_value *val) {
    struct analog_input_data *data = dev->data;
    // const struct analog_input_config *config = dev->config;
    int err;

    if (chan != SENSOR_CHAN_ALL) {
        LOG_DBG("Selected channel is not supported: %d.", chan);
        return -ENOTSUP;
    }
    if (unlikely(!data->ready)) {
        LOG_DBG("Device is not initialized yet");
        return -EBUSY;
    }

    switch ((uint32_t)attr) {
    case ANALOG_INPUT_ATTR_SAMPLING_HZ:
        err = sample_hz_set_value(dev, ANALOG_INPUT_SVALUE_TO_SAMPLING_HZ(*val));
        break;

    case ANALOG_INPUT_ATTR_ENABLE:
        err = enable_set_value(dev, ANALOG_INPUT_SVALUE_TO_ENABLE(*val));
        break;

    case ANALOG_INPUT_ATTR_ACTIVE:
        err = active_set_value(dev, ANALOG_INPUT_SVALUE_TO_ACTIVE(*val));
        break;

    default:
        LOG_ERR("Unknown attribute");
        err = -ENOTSUP;
    }

    return err;
}

static int analog_input_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct analog_input_data *data = dev->data;
    // const struct analog_input_config *config = dev->config;

    if (chan != SENSOR_CHAN_ALL) {
        LOG_DBG("Selected channel is not supported: %d.", chan);
        return -ENOTSUP;
    }
    if (unlikely(!data->ready)) {
        LOG_DBG("Device is not initialized yet");
        return -EBUSY;
    }

    int err = analog_input_report_data(data->dev);
    if (err < 0) {
        LOG_ERR("analog_input_report_data returned %d", err);
        return err;
    }

    return 0;
}

static int analog_input_channel_get(const struct device *dev, enum sensor_channel chan,
                                    struct sensor_value *val) {
    struct analog_input_data *data = dev->data;

    if (unlikely(chan != SENSOR_CHAN_ALL)) {
        LOG_DBG("Selected channel is not supported: %d.", chan);
        return -ENOTSUP;
    }
    if (unlikely(!data->ready)) {
        LOG_DBG("Device is not initialized yet");
        return -EBUSY;
    }

    for (uint8_t i = 0; i < data->rt_ch_len; i++) {
        struct analog_input_io_channel ch_cfg = data->rt_ch[i];
        if (!ch_cfg.report_on_change_only) {
            continue;
        }
        if (i == 0)      val->val1 = data->delta[i];
        else if (i == 1) val->val2 = data->delta[i];
    }

    return 0;
}

static const struct sensor_driver_api analog_input_driver_api = {
    .attr_set = analog_input_attr_set,
    .sample_fetch = analog_input_sample_fetch,
    .channel_get = analog_input_channel_get,
};

#define TRANSFORMED_IO_CHANNEL_ENTRY(node_id)                                                      \
    {                                                                                              \
        .adc_channel = ADC_DT_SPEC_GET_BY_IDX(node_id, 0),                                         \
        .mv_mid = DT_PROP(node_id, mv_mid),                                                        \
        .mv_min_max = DT_PROP(node_id, mv_min_max),                                                \
        .mv_deadzone = DT_PROP(node_id, mv_deadzone),                                              \
        .invert = DT_PROP(node_id, invert),                                                        \
        .report_on_change_only = DT_PROP(node_id, report_on_change_only),                          \
        .scale_multiplier = DT_PROP(node_id, scale_multiplier),                                    \
        .scale_divisor = DT_PROP(node_id, scale_divisor),                                          \
        .evt_type = DT_PROP(node_id, evt_type),                                                    \
        .input_code = DT_PROP(node_id, input_code),                                                \
    }

#define ANIN_IOC_CHILD_LEN_PLUS_ONE(node) 1 +

#define ANALOG_INPUT_DEFINE(n)                                                                     \
    static struct analog_input_data data##n = {                                                    \
    };                                                                                             \
    static const struct analog_input_config config##n = {                                          \
        .sampling_hz = DT_PROP(DT_DRV_INST(n), sampling_hz),                                       \
        .io_channels_len = (DT_FOREACH_CHILD(DT_DRV_INST(n), ANIN_IOC_CHILD_LEN_PLUS_ONE) 0),      \
        .io_channels = { DT_INST_FOREACH_CHILD_SEP(n, TRANSFORMED_IO_CHANNEL_ENTRY, (, )) },       \
    };                                                                                             \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(n, analog_input_init, NULL, &data##n, &config##n, POST_KERNEL,           \
                          CONFIG_SENSOR_INIT_PRIORITY, &analog_input_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ANALOG_INPUT_DEFINE)

#if IS_ENABLED(CONFIG_SHELL)
/* ---- Stick tuner protocol (USB CDC shell) ---- */
/* App sends: stick get | stick mid X Y | stick deadzone N | stick divisor N | stick invert_y 0|1 */

static int cmd_stick_get(const struct shell *shell, size_t argc, char **argv) {
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (!stick_dev) {
		shell_error(shell, "STICK ERR no device");
		return -ENODEV;
	}
	struct analog_input_data *data = stick_dev->data;
	if (!data->ready || data->rt_ch_len < 2) {
		shell_error(shell, "STICK ERR not ready");
		return -EBUSY;
	}
	shell_print(shell,
		    "STICK OK mid_x=%u mid_y=%u deadzone=%u divisor=%u invert_y=%d",
		    data->rt_ch[0].mv_mid, data->rt_ch[1].mv_mid, data->rt_ch[0].mv_deadzone,
		    data->rt_ch[0].scale_divisor, data->rt_ch[1].invert ? 1 : 0);
	return 0;
}

static int cmd_stick_live(const struct shell *shell, size_t argc, char **argv) {
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (!stick_dev) {
		shell_error(shell, "STICK ERR no device");
		return -ENODEV;
	}
	struct analog_input_data *data = stick_dev->data;
	if (!data->ready || data->rt_ch_len < 2) {
		shell_error(shell, "STICK ERR not ready");
		return -EBUSY;
	}
	shell_print(shell, "STICK OK live_x=%d live_y=%d", data->last_mv[0], data->last_mv[1]);
	return 0;
}

static int cmd_stick_mid(const struct shell *shell, size_t argc, char **argv) {
	if (argc != 3 || !stick_dev) {
		shell_error(shell, "STICK ERR usage: stick mid <x> <y>");
		return -EINVAL;
	}
	struct analog_input_data *data = stick_dev->data;
	if (!data->ready || data->rt_ch_len < 2) {
		shell_error(shell, "STICK ERR not ready");
		return -EBUSY;
	}
	data->rt_ch[0].mv_mid = (uint16_t)strtoul(argv[1], NULL, 10);
	data->rt_ch[1].mv_mid = (uint16_t)strtoul(argv[2], NULL, 10);
	shell_print(shell, "STICK OK mid_x=%u mid_y=%u", data->rt_ch[0].mv_mid, data->rt_ch[1].mv_mid);
	return 0;
}

static int cmd_stick_deadzone(const struct shell *shell, size_t argc, char **argv) {
	if (argc != 2 || !stick_dev) {
		shell_error(shell, "STICK ERR usage: stick deadzone <n>");
		return -EINVAL;
	}
	struct analog_input_data *data = stick_dev->data;
	if (!data->ready) {
		shell_error(shell, "STICK ERR not ready");
		return -EBUSY;
	}
	uint8_t dz = (uint8_t)CLAMP(strtoul(argv[1], NULL, 10), 0, 255);
	for (uint8_t i = 0; i < data->rt_ch_len; i++) {
		data->rt_ch[i].mv_deadzone = dz;
	}
	shell_print(shell, "STICK OK deadzone=%u", dz);
	return 0;
}

static int cmd_stick_divisor(const struct shell *shell, size_t argc, char **argv) {
	if (argc != 2 || !stick_dev) {
		shell_error(shell, "STICK ERR usage: stick divisor <n>");
		return -EINVAL;
	}
	struct analog_input_data *data = stick_dev->data;
	if (!data->ready) {
		shell_error(shell, "STICK ERR not ready");
		return -EBUSY;
	}
	uint16_t div = (uint16_t)CLAMP(strtoul(argv[1], NULL, 10), 1, 10000);
	for (uint8_t i = 0; i < data->rt_ch_len; i++) {
		data->rt_ch[i].scale_divisor = div;
	}
	shell_print(shell, "STICK OK divisor=%u", div);
	return 0;
}

static int cmd_stick_invert_y(const struct shell *shell, size_t argc, char **argv) {
	if (argc != 2 || !stick_dev) {
		shell_error(shell, "STICK ERR usage: stick invert_y <0|1>");
		return -EINVAL;
	}
	struct analog_input_data *data = stick_dev->data;
	if (!data->ready || data->rt_ch_len < 2) {
		shell_error(shell, "STICK ERR not ready");
		return -EBUSY;
	}
	data->rt_ch[1].invert = strtoul(argv[1], NULL, 10) != 0;
	shell_print(shell, "STICK OK invert_y=%d", data->rt_ch[1].invert ? 1 : 0);
	return 0;
}

static int cmd_stick_save(const struct shell *shell, size_t argc, char **argv) {
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (!stick_dev) {
		shell_error(shell, "STICK ERR no device");
		return -ENODEV;
	}
	struct analog_input_data *data = stick_dev->data;
	if (!data->ready || data->rt_ch_len < 2) {
		shell_error(shell, "STICK ERR not ready");
		return -EBUSY;
	}
#if IS_ENABLED(CONFIG_SETTINGS)
	int err = stick_save_store(data);
	if (err) {
		shell_error(shell, "STICK ERR save failed %d", err);
		return err;
	}
	shell_print(shell, "STICK OK saved");
	return 0;
#else
	shell_error(shell, "STICK ERR settings disabled");
	return -ENOTSUP;
#endif
}

SHELL_STATIC_SUBCMD_SET_CREATE(stick_cmds,
	SHELL_CMD(get, NULL, "Show calibration", cmd_stick_get),
	SHELL_CMD(live, NULL, "Show live ADC mV", cmd_stick_live),
	SHELL_CMD_ARG(mid, NULL, "Set mid_x mid_y", cmd_stick_mid, 3, 0),
	SHELL_CMD_ARG(deadzone, NULL, "Set deadzone", cmd_stick_deadzone, 2, 0),
	SHELL_CMD_ARG(divisor, NULL, "Set speed divisor (higher=slower)", cmd_stick_divisor, 2, 0),
	SHELL_CMD_ARG(invert_y, NULL, "Invert Y axis 0/1", cmd_stick_invert_y, 2, 0),
	SHELL_CMD(save, NULL, "Save calibration to flash", cmd_stick_save),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(stick, &stick_cmds, "PSP stick calibration", NULL);
#endif /* CONFIG_SHELL */
