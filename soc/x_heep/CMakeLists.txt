# 
# X-HEEP SoC CMakeLists.txt
#

# ============================================================
# Linker Script
# ============================================================
set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/riscv/common/linker.ld CACHE INTERNAL "")

# ============================================================
# Include Directories
# ============================================================
zephyr_include_directories(.)

# ============================================================
# SoC Source Files
# ============================================================
zephyr_sources(
    soc.c
    reset.S
)

# soc_irq.S is only needed if using PLIC for external interrupts
# X-HEEP's OpenTitan PLIC is currently disabled
