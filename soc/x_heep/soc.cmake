# 
# X-HEEP soc.cmake
#

# Set SOC_LINKER_SCRIPT early so Zephyr finds it
set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/riscv/common/linker.ld CACHE INTERNAL "")
