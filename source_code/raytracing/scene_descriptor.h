#ifndef RAYRACING_SCENE_DESCRIPTOR_H_
#define RAYRACING_SCENE_DESCRIPTOR_H_

#include <functional>

#include <geometry/types/vector.h>
#include <geometry/types/varmesh.h>
#include <geometry/types/bezier_surface.h>

struct scene_descriptor
{
	int screen_width;
	int screen_height;
	v3 origin;
	double rotation_angle_e1;
	double rotation_angle_e2;
	double rotation_angle_e3;
	double field_of_view;
	v3 light;	
};

struct varmesh_scene_descriptor : scene_descriptor
{
	varmesh<4> mesh;
	std::function<std::vector<int>(double, double)> mesh_color;
};

struct facetted_surface_scene_descriptor : scene_descriptor
{
	std::vector<std::array<int, 3>>& facettes;
	std::vector<v3>& points;
};

struct subdivided_mesh_scene_descriptor : varmesh_scene_descriptor
{
	subdivided_mesh_scene_descriptor(const varmesh_scene_descriptor &base) : varmesh_scene_descriptor(base)
	{
		meshes_hierarchy = mesh_subdivision_hierarchy(mesh, 6);
	}

	std::vector<varmesh<4>> meshes_hierarchy;
};

struct scene_object {
	varmesh<4> mesh;
	std::function<std::vector<int>(double, double)> mesh_color;
};

struct multiple_surfaces_scene_descriptor : scene_descriptor
{
	std::vector<scene_object> surfaces;
};

#endif // RAYRACING_SCENE_DESCRIPTOR_H_