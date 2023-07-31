#include "raytrace_subdivided_mesh.h"

void trace_ray_with_meshes_hierarchy(std::vector<int>::iterator pixel, v3 ray, subdivided_mesh_scene_descriptor scene)
{
    std::vector<int> intersected_meshes = intersects_mesh(scene.origin, ray, scene.meshes_hierarchy);

    double t = std::numeric_limits<double>::max();
    double shade_factor = 0;

    for (auto index : intersected_meshes)
    {
        v3 bary = remove_dimension(barycentre_of_mesh(scene.meshes_hierarchy[index]));

        double this_t = bary * bary;
        if (this_t < t)
        {
            t = this_t;
            v3 normale = normale_of_mesh(remove_dimension(scene.meshes_hierarchy[index]));
            shade_factor = shade(normale, scene.light);
        }
    }

    *pixel++ = std::round(shade_factor * scene.mesh_color(0, 0)[0]);
    *pixel++ = std::round(shade_factor * scene.mesh_color(0, 0)[1]);
    *pixel++ = std::round(shade_factor * scene.mesh_color(0, 0)[2]);
}
