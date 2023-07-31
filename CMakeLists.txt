cmake_minimum_required(VERSION 3.6)

project(BezierRaytracing)

enable_testing()

INCLUDE_DIRECTORIES(source_code)

add_subdirectory(source_code)
add_subdirectory(unit_tests)
add_subdirectory(integration_tests)






