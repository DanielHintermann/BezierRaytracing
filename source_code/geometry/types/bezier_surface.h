#ifndef geometry_types_bezier_surface_h
#define geometry_types_bezier_surface_h

#include "geometry/types/bezier_curve.h"

template<std::size_t N> v<N> evaluate_polygon_surface(const varmesh<N>& m, double u, double vv)
{
    std::vector<v<N>> colu;

    for (int i = 0; i < m.row_size(); i++)
    {
        colu.push_back(evaluate_polyline(row(m, i), u));
    }

    return evaluate_polyline(colu, vv);
}

template <size_t N> std::vector<v<N>> bezier_curve_on_surface_u(const varmesh<N>& m, double vv)
{
    std::vector<v<N>> result;

    for (int i = 0; i < m.col_size(); i++)
    {
        result.push_back(evaluate_bezier_curve(col(m, i), vv));
    }

    return result;
}

template <size_t N> std::vector<v<N>> bezier_curve_on_surface_v(const varmesh<N>& m, double u)
{
    std::vector<v<N>> result;

    for (int i = 0; i < m.row_size(); i++)
    {
        result.push_back(evaluate_bezier_curve(row(m, i), u));
    }

    return result;
}

template <size_t N> v<N> evaluate_bezier_surface(const varmesh<N>& m, double u, double vv)
{
    std::vector<v<N>> results;

    for (int i = 0; i < m.row_size(); i++)
    {
        std::vector<v<N>> row;
        for (int j = 0; j < m.col_size(); j++)
        {
            row.push_back(m.element(i, j));
        }
        results.push_back(evaluate_bezier_curve(row, u));
    }
    return evaluate_bezier_curve(results, vv);
}

template<std::size_t N> varmesh<N> bezier_surface_quasi_interpolation(const varmesh<N>& m)
{
    varmesh<N> diff_r = m;

    for (int j = 0; j < m.col_size(); j++)
    {
        for (int i = 1; i < m.row_size() - 1; i++)
        {
            diff_r.element(i, j) = 0.25 * (m.element(i - 1, j) + 2 * m.element(i, j) + m.element(i + 1, j));
        }
    }

    varmesh<N> result = diff_r;

    for (int i = 0; i < m.row_size(); i++)
    {
        for (int j = 1; j < m.col_size() - 1; j++)
        {
            result.element(i, j) = 0.25 * (diff_r.element(i, j - 1) + 2 * diff_r.element(i, j) + diff_r.element(i, j + 1));
        }
    }

    return result;
}

template <size_t d> varmesh<d> bezier_surface_insert_control_point_row(const varmesh<d>& m)
{
    varmesh<d> result(m.row_size(), m.col_size() + 1);

    for (int i = 0; i < m.row_size(); i++)
    {
        std::vector<v<d>> current_row;
        for (int j = 0; j < m.col_size(); j++)
        {
            current_row.push_back(m.element(i, j));
        }
        auto divided_row = bezier_curve_insert_control_point(current_row);
        for (int j = 0; j < m.col_size() + 1; j++)
        {
            result.element(i, j) = divided_row[j];
        }
    }

    return result;
}

template <size_t d> varmesh<d> bezier_surface_insert_control_point_column(const varmesh<d>& m)
{
    varmesh<d> result(m.row_size() + 1, m.col_size());

    for (int i = 0; i < m.col_size(); i++)
    {
        std::vector<v<d>> current_column;
        for (int j = 0; j < m.row_size(); j++)
        {
            current_column.push_back(m.element(j, i));
        }
        auto divided_column = bezier_curve_insert_control_point(current_column);
        for (int j = 0; j < m.row_size() + 1; j++)
        {
            result.element(j, i) = divided_column[j];
        }
    }

    return result;
}

template <size_t d> void bezier_clip_surface_by_row(varmesh<d>& m, double umin, double umax)
{
    for (int i = 0; i < m.row_size(); i++)
    {
        auto cur_row = row_ref<d>(m, i);
        bezier_clip_curve<row_ref<d>>(cur_row, umin, umax);
    }
}

template <size_t d> void bezier_clip_surface_by_col(varmesh<d>& m, double umin, double umax)
{
    for (int i = 0; i < m.col_size(); i++)
    {
        auto cur_col = col_ref<d>(m, i);
        bezier_clip_curve(cur_col, umin, umax);
    }
}

template <size_t d> std::pair<varmesh<d>, varmesh<d>> subdivide_bezier_surface_by_row(const varmesh<d>& m, double u)
{
    varmesh<d> left(m.row_size(), m.col_size()), right(m.row_size(), m.col_size());

    for (int i = 0; i < m.row_size(); i++)
    {
        std::vector<std::array<double, d>> current_row(m.col_size());
        for (int j = 0; j < m.col_size(); j++)
        {
            current_row[j] = m.element(i, j);
        }
        auto divided_row = subdivide_bezier_curve(current_row, u);
        for (int j = 0; j < m.col_size(); j++)
        {
            left.element(i, j) = divided_row.first[j];
            right.element(i, j) = divided_row.second[j];
        }
    }

    return std::make_pair(left, right);
}

template <size_t d> std::pair<varmesh<d>, varmesh<d>> subdivide_bezier_surface_by_column(const varmesh<d>& m, double u)
{
    varmesh<d> top(m.row_size(), m.col_size()), bottom(m.row_size(), m.col_size());

    for (int i = 0; i < m.col_size(); i++)
    {
        std::vector<v<d>> current_column(m.row_size());
        for (int j = 0; j < m.row_size(); j++)
        {
            current_column[j] = m.element(j, i);
        }
        auto divided_column = subdivide_bezier_curve(current_column, u);
        for (int j = 0; j < m.row_size(); j++)
        {
            top.element(j, i) = divided_column.first[j];
            bottom.element(j, i) = divided_column.second[j];
        }
    }

    return std::make_pair(top, bottom);
}

template<size_t d> void bezier_clip_surface(varmesh<d>& m, const v2& u, const v2& v)
{
    bezier_clip_surface_by_row(m, u[0], u[1]);
    bezier_clip_surface_by_col(m, v[0], v[1]);
}

varmesh<2> get_bezier_clipped_surface(const varmesh<2>& m, const v2& u, const v2& v);

template<size_t d> std::vector<varmesh<d>> mesh_subdivision_hierarchy(const varmesh<d> m, int steps)
{
    std::vector<varmesh<d>> meshes_hierarchy;

    meshes_hierarchy.push_back(m);

    size_t from_index = 0;
    size_t to_index = 0;
    for (int s = 0; s < steps; s++)
    {
        for (size_t i = from_index; i <= to_index; i++)
        {
            auto div_m = subdivide_bezier_surface_by_row(meshes_hierarchy[i], 0.5);
            auto lm = subdivide_bezier_surface_by_column(div_m.first, 0.5);

            meshes_hierarchy.push_back(lm.first);
            meshes_hierarchy.push_back(lm.second);

            lm = subdivide_bezier_surface_by_column(div_m.second, 0.5);

            meshes_hierarchy.push_back(lm.first);
            meshes_hierarchy.push_back(lm.second);
        }
        from_index = to_index + 1;
        to_index = meshes_hierarchy.size() - 1;
    }

    return meshes_hierarchy;
}

std::pair<v3, v3> evaluate_bezier_surface_derivatives(const varmesh<4>& mesh, double u, double v);

#endif