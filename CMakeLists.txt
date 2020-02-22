cmake_minimum_required(VERSION 3.15)
project(ANN_EA_Robot)

set(CMAKE_CXX_STANDARD 14)

find_package (Eigen3  REQUIRED)
find_package(Python2 COMPONENTS Development NumPy)
find_package(OpenCV REQUIRED)

aux_source_directory(./source SRC)

include_directories( "/usr/include/eigen3" )
include_directories(./include INCLUDE)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(ANN_EA_Robot main.cpp ${INCLUDE} ${SRC} include/Robot.h source/Robot.cpp)

target_include_directories(ANN_EA_Robot PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS})
target_link_libraries(ANN_EA_Robot Python2::Python Python2::NumPy ${OpenCV_LIBS})


