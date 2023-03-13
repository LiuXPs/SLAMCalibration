cmake_minimum_required(VERSION 3.24)
project(snake_calibration)

set(CMAKE_CXX_STANDARD 17)

find_package(Pangolin REQUIRED)
find_package(FMT REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)

include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        "/opt/eigen"
)

set(HEADER_FILES
        include/SnakeCalibr/FrameCal.h
        include/SnakeCalibr/FrameCalibr.h
        include/SnakeCalibr/FrameSLAM.h
        include/SnakeCalibr/SnakeCalibr.h
        )

set(SOURCE_FILES
        src/FrameCal.cpp
        src/FrameCalibr.cpp
        src/FrameSLAM.cpp
        src/SnakeCalibr.cpp
        )

add_executable(${PROJECT_NAME}
        ${HEADER_FILES}
        ${SOURCE_FILES}
        )

target_link_libraries(${PROJECT_NAME}
        fmt::fmt
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES}
        )
