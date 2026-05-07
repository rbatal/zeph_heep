# 
# X-HEEP soc.cmake
#

# Set SOC_LINKER_SCRIPT early so Zephyr finds it
set(SOC_LINKER_SCRIPT ${ZEPHYR_ZEPH_HEEP_MODULE_DIR}/arch/riscv/x_heep/linker.ld CACHE INTERNAL "")
