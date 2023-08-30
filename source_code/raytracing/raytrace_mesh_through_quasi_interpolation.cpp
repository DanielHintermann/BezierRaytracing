#include "raytrace_mesh_through_quasi_interpolation.h"

#include <geometry/linear_algebra/formulas.h>


std::pair<v3, v3> approximate_bezier_surface_derivatives(const varmesh<4>& mesh, double u, double v)
{
    double eps = 1E-8;

    auto y = evaluate_bezier_surface(mesh, u, v);
    auto yu = evaluate_bezier_surface(mesh, u + eps, v);
    auto yv = evaluate_bezier_surface(mesh, u, v + eps);

    double inv_eps = 1.0 / eps;
    
    return std::make_pair(inv_eps * (remove_dimension(yu) - remove_dimension(y)), inv_eps * (remove_dimension(yv) -remove_dimension(y)));
}

std::optional<std::tuple<int, intersection>> get_ray_surface_intersection(v3 ray_origin, v3 ray, multiple_surfaces_scene_descriptor& scene)
{   
    double dist2 = std::numeric_limits<double>::max();
    intersection closest_intersection;
    int intersection_scene_object = -1;

    for (int scene_object_index = 0; scene_object_index < scene.surfaces.size(); scene_object_index++)
    {
        std::optional<intersection> object_intersecton = scene.surfaces[scene_object_index]->get_intersection(ray_origin, ray);

        if (object_intersecton.has_value())
        {
            v3 distvec = object_intersecton->location - ray_origin;

            if (1E-8 < distvec * ray)
            {
                double this_t = distvec * distvec;
                if (this_t < dist2)
                {
                    dist2 = this_t;
                    closest_intersection = *object_intersecton;
                    intersection_scene_object = scene_object_index;
                }
            }
        }
    }

    if (intersection_scene_object < 0)
        return {};

    return { std::make_tuple(intersection_scene_object, closest_intersection) };
}

v3 trace_ray(v3 ray_origin, v3 ray, multiple_surfaces_scene_descriptor& scene, int depth)
{
    ray = normalize(ray);

    auto closest_intersection = get_ray_surface_intersection(ray_origin, ray, scene);

    if (closest_intersection.has_value())
    {
        auto [intersection_scene_object, intersection_parameters] = *closest_intersection;

        bool is_top = (intersection_parameters.normale * ray < 0);

        if (!is_top)
        {
            intersection_parameters.normale = -intersection_parameters.normale;
        }

        auto reflected_ray = reflect(ray, intersection_parameters.normale);

        double shade_factor = shade(intersection_parameters.normale, scene.light - intersection_parameters.location);

        auto mesh_color = scene.surfaces[intersection_scene_object]->mesh_color(intersection_parameters.uv[0], intersection_parameters.uv[1]);

        if (scene.surfaces[intersection_scene_object]->object_material == diffuse)
        {
            if (scene.max_raytracing_recursion > depth)
            {
                auto cumulated_color = v3{ 0, 0, 0 };
                int count = 0;
                for (int i = 0; i < 6; i++)
                {
                    auto sub_ray_dir = normalize(intersection_parameters.normale) + random_unit_vector(intersection_parameters.normale);
                    auto diffuse_ray_color = trace_ray(intersection_parameters.location + 1E-8 * intersection_parameters.normale, sub_ray_dir, scene, depth + 1);
                    cumulated_color = cumulated_color + component_mul(mesh_color, diffuse_ray_color);
                    count++;
                }
                return (1./count) * cumulated_color;
            }
            else
            {
                return v3{ 0.5, 0.5, 0.5 }; // shade_factor* mesh_color;
            }
        }
        else if (scene.surfaces[intersection_scene_object]->object_material == reflective)
        {
            if (scene.max_raytracing_recursion > depth)
            {
                auto reflected_ray_color = trace_ray(intersection_parameters.location + 1E-8 * intersection_parameters.normale, reflected_ray, scene, depth + 1);
                return 0.9 * reflected_ray_color;
            }
            else
            {
                return v3{0.5, 0.5, 0.5}; // shade_factor* mesh_color;
            }
        }
        return shade_factor * mesh_color;
    }

    auto a = 0.5 * (ray[1] + 1.0);
    return (1.0 - a) * v3 { 1.0, 1.0, 1.0 } + a * v3{ 0.5, 0.7, 1.0 };
}