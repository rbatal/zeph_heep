#
# Interrupt Controller CMakeLists
#

zephyr_library_sources_ifdef(CONFIG_X_HEEP_PLIC intc_rv_plic_opentitan.c)
