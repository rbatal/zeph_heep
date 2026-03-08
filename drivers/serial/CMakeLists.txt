#
# Serial Comms CMakeLists
#

zephyr_library_sources_ifdef(CONFIG_UART_X_HEEP uart_x_heep.c)
