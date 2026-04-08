#
# Timer CMakeLists
#

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_XHEEP_RV_TIMER xheep_rv_timer.c)
