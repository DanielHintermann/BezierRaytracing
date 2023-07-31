set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB nurbs_SRC
    "*.h"
    "*.cpp"
)

file(GLOB_RECURSE nurbs_file_io_SRC
    RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    "file_io/*.h"
    "file_io/*.cpp"
)

file(GLOB_RECURSE nurbs_geometry_algorithms_SRC
    RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    "geometry/algorithms/*.h"
    "geometry/algorithms/*.cpp"
)

file(GLOB_RECURSE nurbs_geometry_types_SRC
    RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    "geometry/types/*.h"
    "geometry/types/*.cpp"
)

file(GLOB_RECURSE nurbs_raytracing_SRC
    RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    "raytracing/*.h"
    "raytracing/*.cpp"
)

file(GLOB_RECURSE nurbs_geometry_linear_algebra_SRC
    RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    "geometry/linear_algebra/*.h"
    "geometry/linear_algebra/*.cpp"
)

file(GLOB_RECURSE graphics_SRC
    RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    "graphics/*.h"
    "graphics/*.cpp"
)

source_group(src FILES ${nurbs_SRC})
source_group(src\\file_io FILES ${nurbs_file_io_SRC})
source_group(src\\geometry\\algorithms FILES ${nurbs_geometry_algorithms_SRC})
source_group(src\\geometry\\types FILES ${nurbs_geometry_types_SRC})
source_group(src\\geometry\\linear_algebra FILES ${nurbs_geometry_linear_algebra_SRC})
source_group(src\\raytracing FILES ${nurbs_raytracing_SRC})
source_group(src\\graphics FILES ${graphics_SRC})

add_library(source_code ${nurbs_SRC} ${nurbs_file_io_SRC} ${nurbs_geometry_algorithms_SRC} ${nurbs_geometry_types_SRC} ${nurbs_geometry_linear_algebra_SRC} ${nurbs_raytracing_SRC} ${graphics_SRC})






