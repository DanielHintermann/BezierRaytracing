#include <geometry/types/bezier_surface.h>

varmesh<2> get_bezier_clipped_surface(const varmesh<2>& m, const v2& u, const v2& v)
{
    varmesh<2> result = m;

    bezier_clip_surface(result, u, v);

    return result;
}