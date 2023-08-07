#include "nurbs_raytracing.h"

std::tuple<std::vector<int>, unsigned int, unsigned int> raytrace_scene_with_facetted_surface_multithreaded(facetted_surface_scene_descriptor& scene, int threadcount)
{
    auto trace_ray_with_facetted_surface = [](std::vector<int>::iterator pixel, v3 ray, facetted_surface_scene_descriptor& scene) {
        trace_ray_facetted_surface(pixel, ray, scene);
    };

    return raytrace_scene_multithreaded<facetted_surface_scene_descriptor>(scene, trace_ray_with_facetted_surface, threadcount);
}

std::tuple<std::vector<int>, unsigned int, unsigned int> raytrace_scene_with_meshes_hierarchy_multithreaded(varmesh_scene_descriptor& scene, int threadcount)
{
    subdivided_mesh_scene_descriptor subdivided_scene(scene);
    
    auto trace_ray_with_meshes_hierarchy_functional = [](std::vector<int>::iterator pixel, v3 ray, subdivided_mesh_scene_descriptor& div_scene) {
        trace_ray_with_meshes_hierarchy(pixel, ray, div_scene);
    };

    return raytrace_scene_multithreaded<subdivided_mesh_scene_descriptor>(subdivided_scene, trace_ray_with_meshes_hierarchy_functional, threadcount);
}

std::tuple<std::vector<int>, unsigned int, unsigned int> raytrace_scene_through_quasi_interpolation_multithreaded(varmesh_scene_descriptor& scene, int threadcount)
{
    auto trace_ray_through_quasi_interpolation = [](std::vector<int>::iterator pixel, v3 ray, varmesh_scene_descriptor& scene) {
        trace_ray(pixel, ray, scene);
    };

    return raytrace_scene_multithreaded<varmesh_scene_descriptor>(scene, trace_ray_through_quasi_interpolation, threadcount);
}

std::tuple<std::vector<int>, unsigned int, unsigned int> raytrace_scene_through_quasi_interpolation_multithreaded(multiple_surfaces_scene_descriptor& scene, int threadcount)
{
    auto trace_ray_through_quasi_interpolation = [](std::vector<int>::iterator pixel, v3 ray, multiple_surfaces_scene_descriptor& scene) {
        trace_ray(pixel, ray, scene);
    };

    return raytrace_scene_multithreaded<multiple_surfaces_scene_descriptor>(scene, trace_ray_through_quasi_interpolation, threadcount);
}

