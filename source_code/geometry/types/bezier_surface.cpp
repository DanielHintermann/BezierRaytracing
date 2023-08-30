#include <geometry/types/bezier_surface.h>

varmesh<2> get_bezier_clipped_surface(const varmesh<2>& m, const v2& u, const v2& v)
{
    varmesh<2> result = m;

    bezier_clip_surface(result, u, v);

    return result;
}

std::pair<v3, v3> evaluate_bezier_surface_derivatives(const varmesh<4>& mesh, double u, double v)
{
    auto curve_u = bezier_curve_on_surface_u(mesh, v);
    // TODO
    auto derivative_u = evaluate_rational_bezier_curve_derivative(curve_u, u);

    auto du = remove_last_component(derivative_u);

    auto curve_v = bezier_curve_on_surface_v(mesh, u);
    // TODO
    auto derivative_v = evaluate_rational_bezier_curve_derivative(curve_v, v);

    auto dv = remove_last_component(derivative_v);

    return std::make_pair(du, dv);
}