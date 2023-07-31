#include <cmath>
#include <cfloat>
#include <iostream>
#include <map>

#include "raytrace_facetted_mesh.h"

void trace_ray_facetted_surface(std::vector<int>::iterator pixel, v3 ray, facetted_surface_scene_descriptor scene)
{
    matrix<4, 4> rotation = rotation_in_e2(std::numbers::pi * -51. / 180.) * rotation_in_e3(std::numbers::pi * 5. / 180.) * rotation_in_e1(std::numbers::pi * 21. / 180.);
    v3 ray_rotated = remove_dimension(rotation * add_dimension(ray));
    ray_rotated = normalize(ray_rotated);

    double t = std::numeric_limits<double>::max();
    double shade_factor = 0;
    for (int i = 0; i < scene.facettes.size(); i++)
    {
        double intersect = intersection_with_triangle(scene.origin, ray_rotated, scene.points[scene.facettes[i][0]], scene.points[scene.facettes[i][1]], scene.points[scene.facettes[i][2]]);
        if (intersect < t)
        {
            t = intersect;
            v3 e1 = scene.points[scene.facettes[i][1]] - scene.points[scene.facettes[i][0]];
            v3 e2 = scene.points[scene.facettes[i][2]] - scene.points[scene.facettes[i][0]];
            v3 normale = cross_product(e1, e2);
            shade_factor = shade(normale, scene.light);
        }
    }
    *pixel++ = std::round(shade_factor * 255);
    *pixel++ = std::round(shade_factor * 255);
    *pixel++ = std::round(shade_factor * 255);
}

