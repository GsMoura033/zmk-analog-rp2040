# Local (vendored) analog-input driver with runtime calibration / shell.
zephyr_library()
zephyr_include_directories(modules/zmk-analog-input-driver/include)
zephyr_library_sources_ifdef(CONFIG_ANALOG_INPUT
  modules/zmk-analog-input-driver/src/analog_input.c
)
