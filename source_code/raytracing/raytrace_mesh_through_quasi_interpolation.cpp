#include "raytrace_mesh_through_quasi_interpolation.h"

std::optional<std::tuple<v3, v3, v2>> get_ray_surface_intersection(v3 ray, varmesh_scene_descriptor& scene, double epsilon)
{
    std::vector<v2> intersections = get_intersections_quasi(scene.origin, ray, scene.mesh, epsilon);

    if (0 < intersections.size())
    {
        double dist2 = std::numeric_limits<double>::max();
        int closest = -1;
        v3 distance_vector;
        for (int i = 0; i < intersections.size(); i++)
        {
            v3 distvec = remove_dimension(evaluate_bezier_surface(scene.mesh, intersections[i][0], intersections[i][1])) - scene.origin;

            double this_t = distvec * distvec;
            if (this_t < dist2)
            {
                dist2 = this_t;
                closest = i;
                distance_vector = distvec;
            }
        }

        if (closest < 0)
            return {};

        auto curve_u = bezier_curve_on_surface_u(scene.mesh, intersections[closest][1]);

        auto derivative_u = evaluate_bezier_curve_derivative(curve_u, intersections[closest][0]);

        auto du = remove_last_component(derivative_u);

        auto curve_v = bezier_curve_on_surface_v(scene.mesh, intersections[closest][0]);

        auto derivative_v = evaluate_bezier_curve_derivative(curve_v, intersections[closest][1]);

        auto dv = remove_last_component(derivative_v);

        auto normale = cross_product((du), (dv));

        return { std::make_tuple(distance_vector, normale, v2{ intersections[closest][0], intersections[closest][1] }) };
    }
    return {};
}

void trace_ray(std::vector<int>::iterator pixel, v3 ray, varmesh_scene_descriptor& scene)
{
    auto intersection = get_ray_surface_intersection(ray, scene, 1E-9);

    if (intersection.has_value())
    {
        auto [distance_vector, normale, uv_parameter] = *intersection;

        double shade_factor = shade(normale, scene.light);

        auto mesh_color = scene.mesh_color(uv_parameter[0], uv_parameter[1]);

        *pixel++ = (std::round(shade_factor * mesh_color[0]));
        *pixel++ = (std::round(shade_factor * mesh_color[1]));
        *pixel++ = (std::round(shade_factor * mesh_color[2]));
    }
}