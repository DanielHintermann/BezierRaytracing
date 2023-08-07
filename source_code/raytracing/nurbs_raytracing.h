#ifndef NURBS_RAYTRACING_H_
#define NURBS_RAYTRACING_H_

#include <functional>

#include <geometry/linear_algebra/formulas.h>

#include <graphics/screen_geometry.h>

#include "raytrace_facetted_mesh.h"
#include "raytrace_mesh_through_quasi_interpolation.h"
#include "raytrace_subdivided_mesh.h"

template<class scene_descriptor_type>
void raytrace_scene(std::vector<int>& pixel, synciterator& iter, scene_descriptor_type& scene, std::function<void(std::vector<int>::iterator, v3, scene_descriptor_type&)> trace_ray_functional)
{
    screen_geometry screen(scene.screen_width, scene.screen_height, scene.field_of_view, 0, 0, 0);

    std::pair<int, int> xy;

    while ((xy = iter.next()) != std::make_pair(-1, -1))
    {
        auto ray = screen.get_corresponding_ray(xy.first, xy.second);
        ray = normalize(ray);

        int pixelindex = scene.screen_width * xy.second + xy.first;
        pixelindex *= 3;

        trace_ray_functional(pixel.begin() + pixelindex, ray, scene);
    }
}

template<class scene_descriptor_type> std::tuple<std::vector<int>, unsigned int, unsigned int> raytrace_scene_multithreaded(scene_descriptor_type& scene, std::function<void(std::vector<int>::iterator, v3, scene_descriptor_type&)> trace_ray_functional, int threadcount)
{
    std::vector<int> pixel(scene.screen_width * scene.screen_height * 3, 0);

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    synciterator iter(scene.screen_width, scene.screen_height);

    std::vector<std::thread> threads;
    for (int i = 0; i < threadcount; i++)
    {
        threads.push_back(std::thread([&] { raytrace_scene(pixel, iter, scene, trace_ray_functional); }));
    }

    for (int i = 0; i < threads.size(); i++)
    {
        threads[i].join();
    }

    end = std::chrono::system_clock::now();

    std::chrono::duration<double> elapsed_seconds = end - start;
    std::time_t end_time = std::chrono::system_clock::to_time_t(end);

    std::cout << "\nfinished computation at " << std::ctime(&end_time)
        << "elapsed time: " << elapsed_seconds.count() << "s\n";


    return std::make_tuple(pixel, scene.screen_width, scene.screen_height);
}

std::tuple<std::vector<int>, unsigned int, unsigned int> raytrace_scene_through_quasi_interpolation_multithreaded(varmesh_scene_descriptor& scene, int threadcount);

std::tuple<std::vector<int>, unsigned int, unsigned int> raytrace_scene_with_facetted_surface_multithreaded(facetted_surface_scene_descriptor& scene, int threadcount);

std::tuple<std::vector<int>, unsigned int, unsigned int> raytrace_scene_with_meshes_hierarchy_multithreaded(varmesh_scene_descriptor& scene, int threadcount);

std::tuple<std::vector<int>, unsigned int, unsigned int> raytrace_scene_through_quasi_interpolation_multithreaded(multiple_surfaces_scene_descriptor& scene, int threadcount);


#endif /* NURBS_RAYTRACING_H_ */
