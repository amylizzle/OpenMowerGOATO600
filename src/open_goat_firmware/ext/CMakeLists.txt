cmake_minimum_required(VERSION 3.0)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/xbot_framework/CMakeLists.txt)
  add_subdirectory(xbot_framework EXCLUDE_FROM_ALL)
endif()

# Provide a ulog static stub so firmware can set compile definitions only if missing
# ulog is provided by xbot_framework; do not create a local stub here.

# No-op helper used by firmware CMake
function(target_add_service target name json)
  message(STATUS "Stub: target_add_service(${target} ${name} ${json})")
endfunction()
