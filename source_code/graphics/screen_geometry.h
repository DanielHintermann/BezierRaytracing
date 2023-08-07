#ifndef screen_geometry_h
#define screen_geometry_h

#include <tuple>

#include <geometry/types/vector.h>
#include <geometry/types/matrix.h>

class screen_geometry
{
public:
	screen_geometry(int screen_width, int screen_height, double field_of_view, double a1, double a2, double a3);
	v3 get_corresponding_ray(int x, int y);
	std::tuple<int, int> get_corresponding_screen_pixel(v3 ray);
private:
	int sw;
	int sh;
	double fov;
	double tan_fov_x;
	double tan_fov_y;
	double aspectratio;
	matrix<4, 4> rotation;
	matrix<4, 4> inverse_rotation;
};

#endif