#ifndef raytrace_facetted_mesh_h
#define raytrace_facetted_mesh_h

#include <iostream>
#include <thread>
#include <vector>

#include <raytracing/scene_descriptor.h>
#include <geometry/types/vector.h>
#include <geometry/types/matrix.h>
#include <geometry/types/varmesh.h>
#include <geometry/types/bezier.h>
#include <geometry/algorithms/intersection.h>
#include <geometry/algorithms/quasi_interpolation.h>
#include <file_io/file_io.h>
#include <raytracing/synciterator.h>
#include <graphics/graphics_formulas.h>

void trace_ray_facetted_surface(std::vector<int>::iterator pixel, v3 ray, facetted_surface_scene_descriptor scene);

#endif