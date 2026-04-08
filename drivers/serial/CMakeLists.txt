#
# Serial Comms CMakeLists
#

zephyr_library_sources_ifdef(CONFIG_XHEEP_UART xheep_uart.c)
