cmake_minimum_required(VERSION 3.20)
project(tof_label)

set(CMAKE_CXX_STANDARD 14)
set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/)

add_executable(tof_label
        main.cpp
        TofDepthData.h
        Time.h
        Resolution.h
        CameraMoudleParam.h
        config.h
        CameraParam.h
        file.h
        file.cpp
        type_check.cpp
        type_check.h
        utils.h
        )
# opencv
find_package(OpenCV REQUIRED)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

# yaml
include(${ROOT_DIR}/cmake/find_yaml.cmake)
include_directories(${YAML_CPP_INCLUDE_DIR})
target_link_libraries(tof_label ${YAMLCPP_LIBRARY})
