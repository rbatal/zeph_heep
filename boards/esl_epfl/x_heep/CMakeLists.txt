#
# author: rbatal (rodrigo.batal@gmail.com)
# X-HEEP Board CMakeLists.txt
#
# Post-build step: convert zephyr.elf to the X-HEEP $readmemh hex format.
#
# The new adapted linker script (arch/riscv/x_heep/linker.ld) places __reset
# at the correct hardware boot address (0x180), no boot-vector injection is
# needed now.
# elf2xheep.py is now used just for the ELF to $readmemh format conversion.
# Zephyr's native objcopy cannot produce the .hex so the script is still
# needed integrated into the toolchain.
#
# Output: build/zephyr/zephyr_xheep.hex

find_package(Python3 REQUIRED COMPONENTS Interpreter)

add_custom_command(
  OUTPUT  "${CMAKE_BINARY_DIR}/zephyr/zephyr_xheep.hex"
  COMMAND "${Python3_EXECUTABLE}"
          "${ZEPHYR_ZEPH_HEEP_MODULE_DIR}/elf2xheep.py"
          "${CMAKE_BINARY_DIR}/zephyr/zephyr.elf"
          "${CMAKE_BINARY_DIR}/zephyr/zephyr_xheep.hex"
          --no-boot-vector
  DEPENDS "${CMAKE_BINARY_DIR}/zephyr/zephyr.elf"
  COMMENT "Generating X-HEEP $readmemh hex"
  VERBATIM
)

add_custom_target(xheep_hex ALL
  DEPENDS "${CMAKE_BINARY_DIR}/zephyr/zephyr_xheep.hex"
)
