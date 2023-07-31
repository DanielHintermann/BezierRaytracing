#include "screen_geometry.h"

#include <numbers>
#include <cmath>

screen_geometry::screen_geometry(int screen_width, int screen_height, double field_of_view) : sw(screen_width), sh(screen_height), fov(field_of_view)
{
    tan_fov_y = tan(std::numbers::pi * 0.5 * fov / 180.);
    aspectratio = double(sw) / double(sh);
    tan_fov_x = tan(std::numbers::pi * 0.5 * fov / 180.) * aspectratio;
}

v3 screen_geometry::get_corresponding_ray(int x, int y)
{
    double invWidth = 1. / double(sw);
    double invHeight = 1. / double(sh);

    double xx = (2 * ((x + 0.5) * invWidth) - 1) * tan_fov_x;

    double yy = (1 - 2 * ((y + 0.5) * invHeight)) * tan_fov_y;

    
    return v3{ xx, yy, 1 };
}

std::tuple<int, int> screen_geometry::get_corresponding_screen_pixel(v3 ray)
{
    auto projected_ray = remove_dimension(ray);

    int x = (int)round(-0.5 + 0.5*sw*(projected_ray[0]/tan_fov_x+1));
    int y = (int)round(-0.5 - 0.5 * sh * (projected_ray[1] / tan_fov_y - 1));

	return std::tuple<int, int>(x, y);
}
