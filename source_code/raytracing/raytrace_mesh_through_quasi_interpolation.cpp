#include "raytrace_mesh_through_quasi_interpolation.h"

std::pair<v3, v3> evaluate_bezier_surface_derivatives(const varmesh<4> &mesh, double u, double v)
{
    auto curve_u = bezier_curve_on_surface_u(mesh, v);

    auto derivative_u = evaluate_bezier_curve_derivative(curve_u, u);

    auto du = remove_last_component(derivative_u);

    auto curve_v = bezier_curve_on_surface_v(mesh, u);

    auto derivative_v = evaluate_bezier_curve_derivative(curve_v, v);

    auto dv = remove_last_component(derivative_v);

    return std::make_pair(du, dv);
}

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

            if (0 < distvec * ray)
            {
                double this_t = distvec * distvec;
                if (this_t < dist2)
                {
                    dist2 = this_t;
                    closest = i;
                    distance_vector = distvec;
                }
            }
        }

        if (closest < 0)
            return {};

        auto [du, dv] = evaluate_bezier_surface_derivatives(scene.mesh, intersections[closest][0], intersections[closest][1]);

        auto normale = cross_product((du), (dv));

        return { std::make_tuple(distance_vector, normale, v2{ intersections[closest][0], intersections[closest][1] }) };
    }

    return {};
}

std::optional<std::tuple<int, v3, v3, v2>> get_ray_surface_intersection(v3 ray, multiple_surfaces_scene_descriptor& scene, double epsilon)
{   
    double dist2 = std::numeric_limits<double>::max();
    v2 intersection_uv;
    int intersection_scene_object = -1;
    v3 distance_vector;

    for (int scene_object_index = 0; scene_object_index < scene.surfaces.size(); scene_object_index++)
    {
        std::vector<v2> intersections = get_intersections_quasi(scene.origin, ray, scene.surfaces[scene_object_index].mesh, epsilon);

        if (0 < intersections.size())
        {
            for (int i = 0; i < intersections.size(); i++)
            {
                v3 distvec = remove_dimension(evaluate_bezier_surface(scene.surfaces[scene_object_index].mesh, intersections[i][0], intersections[i][1])) - scene.origin;

                //std::cout << "\nDist vec : " << distvec[0] << ", " << distvec[1] << ", " << distvec[2];

                //auto a = angle(distvec, ray);

                //std::cout << "\nangle : " << 180 * a / std::numbers::pi;

                if (0 < distvec * ray)
                {
                    double this_t = distvec * distvec;
                    if (this_t < dist2)
                    {
                        dist2 = this_t;
                        intersection_uv = intersections[i];
                        distance_vector = distvec;
                        intersection_scene_object = scene_object_index;
                    }
                }
            }
        }
    }

    if (intersection_scene_object < 0)
        return {};

    auto [du, dv] = evaluate_bezier_surface_derivatives(scene.surfaces[intersection_scene_object].mesh, intersection_uv[0], intersection_uv[1]);

    auto normale = cross_product((du), (dv));

    return { std::make_tuple(intersection_scene_object, distance_vector, normale, intersection_uv) };
}

void trace_ray(std::vector<int>::iterator pixel, v3 ray, varmesh_scene_descriptor& scene)
{
    auto intersection = get_ray_surface_intersection(ray, scene, scene.epsilon);

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

void trace_ray(std::vector<int>::iterator pixel, v3 ray, multiple_surfaces_scene_descriptor& scene)
{
    auto intersection = get_ray_surface_intersection(ray, scene, scene.epsilon);

    if (intersection.has_value())
    {
        auto [intersection_scene_object, distance_vector, normale, uv_parameter] = *intersection;

        double shade_factor = shade(normale, scene.light);

        auto mesh_color = scene.surfaces[intersection_scene_object].mesh_color(uv_parameter[0], uv_parameter[1]);

        *pixel++ = (std::round(shade_factor * mesh_color[0]));
        *pixel++ = (std::round(shade_factor * mesh_color[1]));
        *pixel++ = (std::round(shade_factor * mesh_color[2]));
    }
}