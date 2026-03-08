# 
# author: rbatal (rodrigo.batal@gmail.com)
# X-HEEP Board CMakeLists.txt
#
# Configures a post-build step that calls the elf2xheep.py script dedicated to
# format correctly the .hex file in order to be correctly handled by the x_heep
# system (aligns the correct boot address, and reset/init vectors)
#
# Output file location in the Zephyr directory: build/zephyr/zephyr_xheep.hex
#

find_package(Python3 REQUIRED COMPONENTS Interpreter)

# Boot vector address: must match XHEEP_BOOT_ADDRESS in soc/x_heep/soc.h.
# Defined here (not in soc.cmake) because the board subdirectory is configured
# before soc.cmake runs, so soc.cmake variables are not yet in scope.
set(XHEEP_BOOT_ADDR "0x180" CACHE STRING "X-HEEP boot vector address")

add_custom_command(
  OUTPUT  "${CMAKE_BINARY_DIR}/zephyr/zephyr_xheep.hex"
  COMMAND "${Python3_EXECUTABLE}"
          "${ZEPHYR_Z_HEEP_MODULE_DIR}/elf2xheep.py"
          "${CMAKE_BINARY_DIR}/zephyr/zephyr.elf"
          "${CMAKE_BINARY_DIR}/zephyr/zephyr_xheep.hex"
          --boot-addr "${XHEEP_BOOT_ADDR}"
  DEPENDS "${CMAKE_BINARY_DIR}/zephyr/zephyr.elf"
  COMMENT "Generating X-HEEP hex (boot vector @ ${XHEEP_BOOT_ADDR})"
  VERBATIM
)

add_custom_target(xheep_hex ALL
  DEPENDS "${CMAKE_BINARY_DIR}/zephyr/zephyr_xheep.hex"
)
