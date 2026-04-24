#
# GPIO driver CMakeLists
#

zephyr_library_sources_ifdef(CONFIG_XHEEP_GPIO xheep_gpio.c)
