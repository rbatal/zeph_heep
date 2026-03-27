#
# Interrupt Controller CMakeLists
#

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_XHEEP_PLIC xheep_intc_plic.c)
