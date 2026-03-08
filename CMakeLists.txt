# SPDX-License-Identifier: Apache-2.0
# X-HEEP Zephyr Module Root CMakeLists.txt

# This file is required by module.yml (cmake: .)
# It can be empty or contain module-level CMake configuration

if(CONFIG_SOC_FAMILY_X_HEEP)
  add_subdirectory(drivers)
endif()