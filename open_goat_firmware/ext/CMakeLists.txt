cmake_minimum_required(VERSION 3.0)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/xbot_framework/CMakeLists.txt)
  add_subdirectory(xbot_framework EXCLUDE_FROM_ALL)
endif()

# Add embedded third-party libraries placed under ext/, e.g. etl
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/etl/CMakeLists.txt)
  add_subdirectory(etl EXCLUDE_FROM_ALL)
endif()
