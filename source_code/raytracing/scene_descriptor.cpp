#include "scene_descriptor.h"

#include <geometry/algorithms/quasi_interpolation.h>

std::optional<intersection> ray_sphere_intersection(v3 ray_origin, v3 ray, const v3& center, double radius) 
{
    v3 oc = ray_origin - center;
    auto a = length2(ray);
    auto half_b = oc * ray;
    auto c = length2(oc) - radius * radius;
    auto discriminant = half_b * half_b - a * c;

    if (discriminant < 0) 
    {
        return {};
    }
    else 
    {
        double t = (-half_b - sqrt(discriminant)) / a;
        auto intersection_location = t * ray + ray_origin;
        auto normale = intersection_location - center;

        normale = normalize(normale);

        if (length2(center - ray_origin) < radius * radius)
        {
            normale = -normale;
        }

        return { intersection(intersection_location, normale, v2{0, 0}) };
    }
}

std::optional<intersection> get_ray_surface_intersection(v3 ray_origin, v3 ray, const varmesh<4> &mesh, double epsilon)
{
    double dist2 = std::numeric_limits<double>::max();
    v2 intersection_uv;
    v3 distance_vector;

    std::vector<v2> intersections = get_intersections_quasi(ray_origin, ray, mesh, epsilon);

    if (0 < intersections.size())
    {
        for (int i = 0; i < intersections.size(); i++)
        {
            v3 distvec = remove_dimension(evaluate_bezier_surface(mesh, intersections[i][0], intersections[i][1])) - ray_origin;

            if (1E-8 < distvec * ray)
            {
                double this_t = distvec * distvec;
                if (this_t < dist2)
                {
                    dist2 = this_t;
                    intersection_uv = intersections[i];
                    distance_vector = distvec;
                }
            }
        }
    }

    if (! (dist2 < std::numeric_limits<double>::max()))
        return {};

    auto [du, dv] = evaluate_bezier_surface_derivatives(mesh, intersection_uv[0], intersection_uv[1]);

    auto normale = cross_product((du), (dv));

    return { intersection(distance_vector + ray_origin + 1E-8 * normale, normale, intersection_uv) };
}


scene_object::scene_object(std::function<v3(double, double)> t, material m) : mesh_color(t), object_material(m)
{
}

varmesh_scene_object::varmesh_scene_object(const varmesh<4>& m, std::function<v3(double, double)> texture, material mo) : scene_object(texture, mo), mesh{m}
{
}

//sphere_scene_object(v3 c, double r, std::function<v3(double, double)> t, material m) : scene_object(t, m), sphere_centre(c), sphere_radius(r)
//{
//
//}

std::optional<intersection> varmesh_scene_object::get_intersection(v3 ray_origin, v3 ray_direction)
{
	return get_ray_surface_intersection(ray_origin, ray_direction, this->mesh, 1E-8);
}

sphere_scene_object::sphere_scene_object(v3 c, double r, std::function<v3(double, double)> f, material m) : scene_object(f, m), sphere_radius{r}, sphere_centre{c}
{
}

std::optional<intersection> sphere_scene_object::get_intersection(v3 ray_origin, v3 ray_direction)
{
    auto intersection = ray_sphere_intersection(ray_origin, ray_direction, this->sphere_centre, this->sphere_radius);

    return intersection;
}
