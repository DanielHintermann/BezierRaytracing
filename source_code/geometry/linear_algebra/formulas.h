#ifndef geometry_linear_algebra_formulas_h
#define geometry_linear_algebra_formulas_h

#include <geometry/types/vector.h>
#include <geometry/types/varmesh.h>

double angle(v3, v3);

v2 invBilinear(const v2& p, const v2& a, const v2& b, const v2& c, const v2& d);

double cross(const v2& a, const v2& b);

std::vector<double> solve_linear_form(const v2& p, const v2& a, const v2& b, const v2& c, const v2& d);

v2 rotate_counter_clockwise_90(const v2& p);

v3 hesse_from(const v2& p, const v2& q);
v3 unnormalized_hesse_from(const v2& p, const v2& q);
v3 cross_product(const v3& u, const v3& v);

v4 vertical_plane_of_ray(const v3& origin, const v3& dir);
v4 horizontal_plane_of_ray(const v3& origin, const v3& dir);
v4 distance_plane_of_ray(const v3& origin, const v3& direction);

v3 project_onto_ray(const v3& origin, const v3& dir, const v3& point);

bool angle_less(const v2& p, const v2& q);

inline double cross_product_scale(const v2& u, const v2& v)
{
    return u[0] * v[1] - u[1] * v[0];
}

inline bool compare_polar(const v2& p, const v2& q)
{
    const v2 e1{ {1., 0.} };

    double p_e1 = cross_product_scale(e1, p);
    double q_e1 = cross_product_scale(e1, q);

    if (0 < p_e1 && 0 > q_e1)
    {
        return true;
    }

    if (0 > p_e1 && 0 < q_e1)
    {
        return false;
    }

    if (0 == p_e1 && 0 == q_e1)
    {
        return false;
    }

    if (0 == p_e1)
    {
        return true;
    }

    if (0 == q_e1)
    {
        return false;
    }

    double pq = cross_product_scale(p, q);

    return 0 < pq;
}

inline bool counter_clockwise_angle_not_greater_180(const v2& p, const v2& q)
{
    return 0 <= cross_product_scale(p, q);
}

varmesh<2> project_mesh(const varmesh<4>& m, const v4& plane1, const v4& plane2);


v3 normale_of_mesh(const varmesh<3>& m);

#endif // geometry_linear_algebra_formulas_h
