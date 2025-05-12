cmake_minimum_required (VERSION 3.10)
project ("Manipulator kinematics")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(manipulator_knimatics
src/main.cpp 
src/kinematics.cpp
src/quaternion.cpp
)
