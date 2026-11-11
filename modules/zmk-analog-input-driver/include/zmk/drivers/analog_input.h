/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 * Copyright (c) 2026 GsMoura033 (runtime calibration)
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef ZEPHYR_INCLUDE_ANALOG_INPUT_H_
#define ZEPHYR_INCLUDE_ANALOG_INPUT_H_

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ANALOG_INPUT_MAX_CHANNELS 4

struct analog_input_io_channel {
	struct adc_dt_spec adc_channel;
	uint16_t mv_mid;
	uint16_t mv_min_max;
	uint16_t mv_deadzone;
	bool invert;
	bool report_on_change_only;
	uint16_t scale_multiplier;
	uint16_t scale_divisor;
	uint8_t evt_type;
	uint8_t input_code;
};

struct analog_input_data {
	const struct device *dev;
	struct adc_sequence as;
#if CONFIG_ADC_ASYNC
	struct k_poll_signal async_sig;
	struct k_poll_event async_evt;
#endif
	uint16_t *as_buff;
	int32_t *delta;
	int32_t *prev;
	struct k_work_delayable init_work;
	int async_init_step;
	bool ready;

	uint32_t sampling_hz;
	bool enabled;
	bool actived;

	struct k_work sampling_work;
	struct k_timer sampling_timer;
	int err;

	/* Runtime-tunable copy of DT channel config (for stick tuner app) */
	uint8_t rt_ch_len;
	struct analog_input_io_channel rt_ch[ANALOG_INPUT_MAX_CHANNELS];
	int32_t last_raw[ANALOG_INPUT_MAX_CHANNELS];
	int32_t last_mv[ANALOG_INPUT_MAX_CHANNELS];
};

struct analog_input_config {
	uint32_t sampling_hz;
	uint8_t io_channels_len;
	struct analog_input_io_channel io_channels[];
};

#define ANALOG_INPUT_SVALUE_TO_SAMPLING_HZ(svalue) ((uint32_t)(svalue).val1)
#define ANALOG_INPUT_SVALUE_TO_ENABLE(svalue) ((uint32_t)(svalue).val1)
#define ANALOG_INPUT_SVALUE_TO_ACTIVE(svalue) ((uint32_t)(svalue).val1)

enum analog_input_attribute {
	ANALOG_INPUT_ATTR_SAMPLING_HZ,
	ANALOG_INPUT_ATTR_ENABLE,
	ANALOG_INPUT_ATTR_ACTIVE,
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_ANALOG_INPUT_H_ */
