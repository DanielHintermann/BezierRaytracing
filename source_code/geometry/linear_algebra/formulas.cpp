#include "formulas.h"

v2 invBilinear(const v2& p, const v2& a, const v2& b, const v2& c, const v2& d)
{
    if (p == a)
    {
        return v2{ {0, 0} };
    }

    if (p == b)
    {
        return v2{ {1, 0} };
    }

    if (p == c)
    {
        return v2{ {1, 1} };
    }

    if (p == d)
    {
        return v2{ {0, 1} };
    }

    v2 e = b - a;
    v2 f = d - a;
    v2 g = a - b + c - d;
    v2 h = p - a;

    double k2 = cross(g, f);
    double k1 = cross(e, f) + cross(h, g);
    double k0 = cross(h, e);

    double w = k1 * k1 - 4.0 * k0 * k2;
    if (w < 0.0) return v2{ {-1, -1} };
    w = sqrt(w);

    double v1 = (-k1 - w) / (2.0 * k2);
    double u1 = (h[0] - f[0] * v1) / (e[0] + g[0] * v1);

    double v2 = (-k1 + w) / (2.0 * k2);
    double u2 = (h[0] - f[0] * v2) / (e[0] + g[0] * v2);

    double uu = u1;
    double vv = v1;

    if (vv < 0.0 || vv>1.0 || uu < 0.0 || uu>1.0) { uu = u2;   vv = v2; }
    if (vv < 0.0 || vv>1.0 || uu < 0.0 || uu>1.0) { uu = -1.0; vv = -1.0; }

    return v<2>{ {uu, vv}};
}

double cross(const v2& a, const v2& b) { return a[0] * b[1] - a[1] * b[0]; }

std::vector<double> solve_quadratic_polynom(double a, double b, double c)
{
    std::vector<double> solutions;

    if (0 == a)
    {
        if (0 != b)
        {
            solutions.push_back(-c / b);
        }
    }
    else
    {
        double disc = b * b - 4 * a * c;


        if (0 == disc)
        {
            solutions.push_back(-b / (2 * a));

        }

        if (0 < disc)
        {
            double sdisc = sqrt(disc);
            solutions.push_back((-b - sdisc) / (2 * a));
            solutions.push_back((-b + sdisc) / (2 * a));
        }
    }
    return solutions;
}

std::vector<double> solve_linear_form(const v2& p, const v2& a, const v2& b, const v2& c, const v2& d)
{
    v2 ab = b - a;
    v2 cd = d - c;
    v2 ac = c - a;
    v2 ap = p - a;

    v2 abcd = cd - ab;

    v2 rac = rotate_counter_clockwise_90(ac);

    v2 rabcd = rotate_counter_clockwise_90(abcd);

    double pc = rac * ap;

    double pb = rabcd * ap - ab * rac;

    double pa = -ab * rabcd;

    auto sols = solve_quadratic_polynom(pa, pb, pc);

    std::vector<double> result;
    for (int i = 0; i < sols.size(); i++)
    {
        if (-1e-9 <= sols[i] && 1 + 1e-9 >= sols[i])
        {
            result.push_back(sols[i]);
        }
    }

    return result;
}

v2 rotate_counter_clockwise_90(const v2& p)
{
    return v2{ {-p[1], p[0]} };
}

v3 hesse_from(const v2& p, const v2& q)
{
    v3 result{ {0, 0, 0} };

    v2 diff = q - p;

    v2 normal = normalize(rotate_counter_clockwise_90(diff));

    if (0 > normal * p)
    {
        normal = -normal;
    }

    result = { {normal[0], normal[1], -(normal * p)} };

    return result;
}

v3 unnormalized_hesse_from(const v2& p, const v2& q)
{
    v3 result{ {0, 0, 0} };

    v2 diff = q - p;

    v2 normal = rotate_counter_clockwise_90(diff);

    if (0 > normal * p)
    {
        normal = -normal;
    }

    result = { {normal[0], normal[1], -(normal * p)} };

    return result;
}

v3 cross_product(const v3& u, const v3& v)
{
    return v3{ {u[1] * v[2] - u[2] * v[1], -u[0] * v[2] + u[2] * v[0], u[0] * v[1] - u[1] * v[0]} };
}

v4 vertical_plane_of_ray(const v3& origin, const v3& dir)
{
    v3 n{ {dir[2], 0, -dir[0]} };
    n = normalize(n);
    double d = -(origin * n);

    return v4{ {n[0], n[1], n[2], d} };
}

v4 horizontal_plane_of_ray(const v3& origin, const v3& dir)
{
    v3 n{ {0, dir[2], -dir[1]} };
    n = normalize(n);
    double d = -(origin * n);

    return v4{ {n[0], n[1], n[2], d} };
}

v4 distance_plane_of_ray(const v3& origin, const v3& direction)
{
    return v4{ {direction[0], direction[1], direction[2], -origin * direction} };
}

v3 project_onto_ray(const v3& origin, const v3& dir, const v3& point)
{
    v3 dir_normalized = normalize(dir);
    return origin + (dir_normalized * (point - origin)) * dir_normalized;
}

int quadrant(const v2& p)
{
    if (0 > p[0])
    {
        if (0 > p[1])
        {
            return 5;
        }
        if (0 == p[1])
        {
            return 4;
        }
        if (0 < p[1])
        {
            return 3;
        }
    }
    if (0 == p[0])
    {
        if (0 > p[1])
        {
            return 6;
        }
        if (0 == p[1])
        {
            return 0;
        }
        if (0 < p[1])
        {
            return 2;
        }
    }
    if (0 < p[0])
    {
        if (0 > p[1])
        {
            return 7;
        }
        if (0 == p[1])
        {
            return 0;
        }
        if (0 < p[1])
        {
            return 1;
        }
    }

    return -1;
}

bool angle_less(const v2& p, const v2& q)
{
    int quadrant_p = quadrant(p);
    int quadrant_q = quadrant(q);

    if (quadrant_q > quadrant_p)
    {
        return true;
    }

    if (quadrant_q < quadrant_p)
    {
        return false;
    }

    // x or y coordinate is 0 and same quadrant
    if (0 == quadrant_p % 2)
    {
        return true;
    }

    double ratio_p = p[1] / p[0];
    double ratio_q = q[1] / q[0];

    return ratio_q > ratio_p;
}

v3 normale_of_mesh(const varmesh<3>& m)
{
    const v3& origin = m.element(0, 0);
    const v3& e01 = m.element(0, m.col_size() - 1);
    const v3& e10 = m.element(m.row_size() - 1, 0);

    return cross_product(e01 - origin, e10 - origin);
}

varmesh<2> project_mesh(const varmesh<4>& m, const v4& plane1, const v4& plane2)
{
    varmesh<2> result(m.row_size(), m.col_size());

    for (int i = 0; i < m.row_size(); i++)
    {
        for (int j = 0; j < m.col_size(); j++)
        {
            result.element(i, j) = v2{ {m.element(i, j) * plane1, m.element(i, j) * plane2} };
        }
    }

    return result;
}

double angle(v3 x, v3 y)
{
    auto lx = length(x);
    auto ly = length(y);

    double ang = 0;

    if (lx > 0 && ly > 0)
    {
        auto cos_ang = x*y / (lx*ly);

        ang = acos(cos_ang);
    }

    return ang;
}