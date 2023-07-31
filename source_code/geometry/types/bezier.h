#ifndef bezier_h
#define bezier_h

#include "vector.h"
#include "geometry/linear_algebra/formulas.h"
#include "matrix.h"
#include "varmesh.h"

#include <assert.h>

template<std::size_t N> v<N> convex_combination(const v<N>& p, const v<N>& q, double weight)
{
    return (1 - weight) * p + weight * q;
}

template<std::size_t N> std::vector<v<N>> convex_combine_points(const std::vector<v<N>>& points, double u)
{
    std::vector<v<N>> result;

    for (int i = 0; i < points.size() - 1; i++)
    {
        result.push_back(convex_combination(points[i], points[i + 1], u));
    }

    return result;
}

template <std::size_t N>
std::vector<v<N>> centred_second_order_differences(const std::vector<v<N>>& points) {
    std::vector<v<N>> result;

    for (int i = 1; i < points.size() - 1; i++)
    {
        result.push_back(points[i - 1] - 2 * points[i] + points[i + 1]);
    }

    return result;
}

double bezier_nairn_coefficient(int degree);

double bezier_z_coefficient(int degree);

std::vector<v2> solve_linear_form(const v2 &y, const varmesh<2> &m, double epsilon = 1e-9);

#endif /* bezier_h */
