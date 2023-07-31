#ifndef screen_geometry_h
#define screen_geometry_h

#include <tuple>

#include <geometry/types/vector.h>

class screen_geometry
{
public:
	screen_geometry(int screen_width, int screen_height, double field_of_view);
	v3 get_corresponding_ray(int x, int y);
	std::tuple<int, int> get_corresponding_screen_pixel(v3 ray);
private:
	int sw;
	int sh;
	double fov;
	double tan_fov_x;
	double tan_fov_y;
	double aspectratio;
};

#endif