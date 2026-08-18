cmake_minimum_required(VERSION 3.16)
# Set the project name
set(CMAKE_PROJECT_NAME open_goat)

# Set C++ standard globally before project initialization
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# Include git version script
include("cmake/GetGitVersion.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)


# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Generate git version header file
set(GENERATED_INCLUDE_DIR ${CMAKE_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${GENERATED_INCLUDE_DIR})
get_git_version(${GENERATED_INCLUDE_DIR})

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add generated include directory for git_version.h
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${GENERATED_INCLUDE_DIR})

target_compile_options(${CMAKE_PROJECT_NAME} PUBLIC -Wall -Wextra -Werror)

# Add lib subdirectory
add_subdirectory(ext)
target_compile_definitions(ulog PUBLIC ULOG_ENABLED)


# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        src/main.cpp
        src/services/imu_service/imu_service.cpp
        src/services/power_service/power_service.cpp
        src/services/emergency_service/emergency_service.cpp
        src/services/diff_drive_service/diff_drive_service.cpp
        src/services/mower_service/mower_service.cpp
        src/services/gps_service/gps_service.cpp
        src/services/high_level_service/high_level_service.cpp
        # GPS driver
        src/drivers/gps/gps_driver.cpp
        src/drivers/gps/ublox_gps_driver.cpp
        src/drivers/gps/nmea_gps_driver.cpp
        ext/minmea/minmea.c
        # MCU driver
        src/drivers/mcu/dispatcher.cpp
        # Power driver
        src/drivers/power/power_driver.cpp
        # Emergency driver
        src/drivers/emergency/emergency_driver.cpp
        # IMU driver
        src/drivers/imu/imu_driver.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined include paths
        src
        ext/minmea
        services
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC
        # Add user defined libraries
        etl
        lwjson
)

target_add_service(${CMAKE_PROJECT_NAME} ImuService ${CMAKE_CURRENT_SOURCE_DIR}/../src/open_mower_ros/services/imu_service.json)
target_add_service(${CMAKE_PROJECT_NAME} PowerService ${CMAKE_CURRENT_SOURCE_DIR}/../src/open_mower_ros/services/power_service.json)
target_add_service(${CMAKE_PROJECT_NAME} EmergencyService ${CMAKE_CURRENT_SOURCE_DIR}/../src/open_mower_ros/services/emergency_service.json)
target_add_service(${CMAKE_PROJECT_NAME} DiffDriveService ${CMAKE_CURRENT_SOURCE_DIR}/../src/open_mower_ros/services/diff_drive_service.json)
target_add_service(${CMAKE_PROJECT_NAME} MowerService ${CMAKE_CURRENT_SOURCE_DIR}/../src/open_mower_ros/services/mower_service.json)
target_add_service(${CMAKE_PROJECT_NAME} GpsService ${CMAKE_CURRENT_SOURCE_DIR}/../src/open_mower_ros/services/gps_service.json)
target_add_service(${CMAKE_PROJECT_NAME} HighLevelService ${CMAKE_CURRENT_SOURCE_DIR}/../src/open_mower_ros/services/high_level_service.json)

set_target_properties(${CMAKE_PROJECT_NAME}
        PROPERTIES SUFFIX ".elf")

