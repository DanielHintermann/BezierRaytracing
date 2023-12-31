#ifndef raytrace_mesh_through_quasi_interpolation_h
#define raytrace_mesh_through_quasi_interpolation_h

#include <iostream>
#include <thread>
#include <vector>
#include <optional>

#include <raytracing/scene_descriptor.h>
#include <geometry/types/vector.h>
#include <geometry/types/matrix.h>
#include <geometry/types/varmesh.h>
#include <geometry/types/bezier.h>
#include <geometry/types/bezier_curve.h>
#include <geometry/types/bezier_surface.h>
#include <geometry/algorithms/intersection.h>
#include <geometry/algorithms/quasi_interpolation.h>
#include <file_io/file_io.h>
#include <raytracing/synciterator.h>
#include <graphics/graphics_formulas.h>

std::optional<std::tuple<v3, v3, v2>> get_ray_surface_intersection(v3 ray, varmesh_scene_descriptor& scene, double epsilon);
std::optional<std::tuple<int, v3, v3, v2>> get_ray_surface_intersection(v3 ray, multiple_surfaces_scene_descriptor& scene, double epsilon);

void trace_ray(std::vector<int>::iterator pixel, v3 ray, varmesh_scene_descriptor& scene);

void trace_ray(std::vector<int>::iterator pixel, v3 ray, multiple_surfaces_scene_descriptor& scene);


#endif