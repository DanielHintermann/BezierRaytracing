#ifndef geometry_types_bezier_curve_h
#define geometry_types_bezier_curve_h

#include <geometry/types/bezier.h>

template<std::size_t N> v<N> evaluate_polyline(const std::vector<v<N>>& control_points, double u)
{
    int index = floor((control_points.size() - 1) * u);

    if (index == control_points.size() - 1)
    {
        return control_points[control_points.size() - 1];
    }

    u -= ((double)index) / (control_points.size() - 1);
    u *= control_points.size() - 1;

    return convex_combination(control_points[index], control_points[index + 1], u);
}

template<std::size_t N> v<N> evaluate_bezier_curve(const std::vector<v<N>>& control_points, double u)
{
    std::vector<v<N>> points{ control_points };

    while (1 < points.size())
    {
        points = convex_combine_points(points, u);
    }

    return points[0];
}

template<std::size_t N> v<N> evaluate_bezier_curve_derivative(const std::vector<v<N>>& control_points, double u)
{
    std::vector<v<N>> diffs;

    for (int i = 0; i < control_points.size() - 1; i++)
    {
        diffs.push_back(control_points[i + 1] - control_points[i]);
    }

    return evaluate_bezier_curve(diffs, u);
}


template<std::size_t N> std::vector<v<N>> bezier_curve_insert_control_point(const std::vector<v<N>>& control_points)
{
    std::vector<v<N>> result;

    result.push_back(control_points[0]);

    for (int i = 0; i < control_points.size() - 1; i++)
    {
        double u = (1. + (double)i) / (control_points.size());
        result.push_back(convex_combination(control_points[i], control_points[i + 1], 1 - u));
    }

    result.push_back(control_points[control_points.size() - 1]);

    return result;
}

template<std::size_t N> std::vector<v<N>> bezier_curve_quasi_interpolation(const std::vector<v<N>>& control_points)
{
    std::vector<v<N>> result;

    result.push_back(control_points[0]);

    for (int i = 1; i < control_points.size() - 1; i++)
    {
        result.push_back(0.25 * (control_points[i - 1] + 2 * control_points[i] + control_points[i + 1]));
    }

    result.push_back(control_points[control_points.size() - 1]);

    return result;
}



template<std::size_t N> std::pair<std::vector<v<N>>, std::vector<v<N>>> subdivide_bezier_curve(const std::vector<v<N>>& control_points, double u)
{
    std::vector<v<N>> left, right, points{ control_points };

    while (1 <= points.size())
    {
        left.push_back(points.front());
        right.insert(right.begin(), points.back());

        points = convex_combine_points(points, u);
    }

    return std::make_pair(left, right);
}

template<typename C> void bezier_clip_curve(C& control_points, double umin, double umax)
{
    if (umax < 1)
    {
        for (int i = 1; i < control_points.size(); i++)
        {
            for (int j = control_points.size() - 1; j >= i; j--)
            {
                control_points[j] = convex_combination(control_points[j - 1], control_points[j], umax);
            }
        }
    }

    if (0 < umin)
    {
        umin = umin / umax;
        for (int i = control_points.size() - 1; i > 0; i--)
        {
            for (int j = 0; j < i; j++)
            {
                control_points[j] = convex_combination(control_points[j], control_points[j + 1], umin);
            }
        }
    }
}

#endif // geometry_types_bezier_curve_h
