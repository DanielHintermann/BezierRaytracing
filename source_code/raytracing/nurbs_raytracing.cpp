#include "nurbs_raytracing.h"

#include <random>

double gamma_correction(double v, double g)
{
    return v < 0 ? 0 : pow(v, 1./g);
}

std::tuple<std::vector<int>, unsigned int, unsigned int> raytrace_scene_with_facetted_surface_multithreaded(facetted_surface_scene_descriptor& scene, int threadcount)
{
    auto trace_ray_with_facetted_surface = [](std::vector<int>::iterator pixel, v3 ray, facetted_surface_scene_descriptor& scene) {
        trace_ray_facetted_surface(pixel, ray, scene);
    };

    return raytrace_scene_multithreaded<facetted_surface_scene_descriptor>(scene, trace_ray_with_facetted_surface, threadcount);
}

std::tuple<std::vector<int>, unsigned int, unsigned int> raytrace_scene_through_quasi_interpolation_multithreaded(multiple_surfaces_scene_descriptor& scene, int threadcount)
{
    static std::random_device rd;  // Will be used to obtain a seed for the random number engine
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<> dis(-0.5, 0.5);

    auto trace_ray_through_quasi_interpolation = [](std::vector<int>::iterator pixel, v3 ray, multiple_surfaces_scene_descriptor& scene) {
        auto [x, y] = scene.screen.get_corresponding_screen_pixel(ray);

        v3 cummulated_color{0, 0, 0};
        int count = 0;
        for (int i = 0; i < 2; i++)
        {
            v2 randomized_pixel = v2{ x + dis(gen), y + dis(gen) };

            auto randomized_ray = scene.screen.get_corresponding_ray(randomized_pixel[0], randomized_pixel[1]);

            auto color = trace_ray(scene.origin, randomized_ray, scene, 0);
            cummulated_color = cummulated_color + color;
            count++;
        }

        double gamma = 1.5;
        double inv_count = 1. / count;

        *pixel++ = (std::round(255 * gamma_correction(inv_count * cummulated_color[0], gamma)));
        *pixel++ = (std::round(255 * gamma_correction(inv_count * cummulated_color[1], gamma)));
        *pixel++ = (std::round(255 * gamma_correction(inv_count * cummulated_color[2], gamma)));
    };

    return raytrace_scene_multithreaded<multiple_surfaces_scene_descriptor>(scene, trace_ray_through_quasi_interpolation, threadcount);
}

