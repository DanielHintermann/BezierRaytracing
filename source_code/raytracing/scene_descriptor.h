#ifndef RAYRACING_SCENE_DESCRIPTOR_H_
#define RAYRACING_SCENE_DESCRIPTOR_H_

#include <functional>
#include <optional>

#include <geometry/types/vector.h>
#include <geometry/types/varmesh.h>
#include <geometry/types/bezier_surface.h>
#include <graphics/screen_geometry.h>

enum material
{
	diffuse,
	reflective
};

struct intersection {
	v3 location;
	v3 normale;
	v2 uv;
};

struct intersectable {
	virtual std::optional<intersection> get_intersection(v3 ray_origin, v3 ray_direction) = 0;
};

struct scene_descriptor
{
	screen_geometry screen;
	v3 origin;
	v3 light;	
	int max_raytracing_recursion = 10;
};

struct varmesh_scene_descriptor : scene_descriptor
{
	varmesh<4> mesh;
	std::function<v3(double, double)> mesh_color;
	double epsilon;
};

struct facetted_surface_scene_descriptor : scene_descriptor
{
	std::vector<std::array<int, 3>>& facettes;
	std::vector<v3>& points;
};

struct scene_object : intersectable
{
	scene_object(std::function<v3(double, double)>, material);

	std::function<v3(double, double)> mesh_color;
	material object_material;
};

struct varmesh_scene_object : public scene_object {
	varmesh_scene_object(const varmesh<4>&, std::function<v3(double, double)>, material);

	std::optional<intersection> get_intersection(v3 ray_origin, v3 ray_direction) override;

	varmesh<4> mesh;
};

struct sphere_scene_object : public scene_object {
	sphere_scene_object(v3 c, double r, std::function<v3(double, double)> f, material m);

	v3 sphere_centre;
	double sphere_radius;

	std::optional<intersection> get_intersection(v3 ray_origin, v3 ray_direction) override;
};

struct multiple_surfaces_scene_descriptor : scene_descriptor
{
	std::vector<std::shared_ptr<scene_object>> surfaces;
	double epsilon;
};

#endif // RAYRACING_SCENE_DESCRIPTOR_H_