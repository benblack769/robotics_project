cmake_minimum_required(VERSION 3.5)

project(game_theory3 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(game_theory3 main.cpp)

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(game_theory3 PUBLIC OpenMP::OpenMP_CXX)
endif()
