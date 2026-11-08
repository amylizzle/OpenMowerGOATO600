cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(goat_ros)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        std_msgs
        cv_bridge
        image_transport
        camera_info_manager
)
find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

# The camera's shared memory layout and control protocol are reverse
# engineered, and open_goat_firmware's camera service reads the same segment.
# Compile that driver in here rather than keeping a second copy of the format
# in sync by hand - it depends on nothing but libstdc++ and POSIX.
set(GOAT_FIRMWARE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../open_goat_firmware/src)

add_executable(goat_camera_node
        src/goat_camera_node.cpp
        ${GOAT_FIRMWARE_DIR}/drivers/camera/camera_driver.cpp
)
target_include_directories(goat_camera_node PRIVATE ${GOAT_FIRMWARE_DIR})
add_dependencies(goat_camera_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(goat_camera_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS goat_camera_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
