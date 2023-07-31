#ifndef raytrace_subdivided_mesh_h
#define raytrace_subdivided_mesh_h

#include <iostream>
#include <thread>
#include <vector>

#include <raytracing/scene_descriptor.h>
#include <geometry/types/vector.h>
#include <geometry/types/matrix.h>
#include <geometry/types/bezier.h>
#include <geometry/algorithms/intersection.h>
#include <geometry/algorithms/quasi_interpolation.h>
#include <geometry/types/varmesh.h>
#include <file_io/file_io.h>
#include <raytracing/synciterator.h>
#include <graphics/graphics_formulas.h>

void trace_ray_with_meshes_hierarchy(std::vector<int>::iterator pixel, v3 ray, subdivided_mesh_scene_descriptor scene);

#endif

