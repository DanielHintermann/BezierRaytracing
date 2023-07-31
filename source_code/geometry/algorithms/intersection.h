#ifndef intersection_hpp
#define intersection_hpp

#include <deque>
#include <numbers>
#include <vector>

#include <geometry/linear_algebra/formulas.h>
#include <geometry/types/varmesh.h>
#include <geometry/types/bezier.h>


double intersection_with_triangle(const v3 &origin, const v3 &direction, const v3 &u, const v3 &v, const v3 &w);
bool intersects_convex_hull(const v3 &origin, const v3 &dir, const std::vector<v3> &points);
bool is_origin_in_convex_hull_atan2(std::vector<v2> &points);
bool is_origin_in_convex_hull(std::vector<v2> &points);

bool intersects_e1_halfline(const v2& p, const v2 q);
bool origin_inside_polygon(const std::vector<v2> &polygon);
bool inside_polygon(const v2& point, const std::vector<v2> &polygon);
v2 closest_point_to_origin_of_segment(const v2& p, const v2& q);
bool origin_circle_overlaps_segment(const v2& p, const v2& q, double radius2);
bool circle_overlaps_polygon(double radius, const std::vector<v<2>> &polygon);


std::vector<int> intersects_mesh(const v3& origin, const v3& ray, const std::vector<varmesh<4>>& meshes_hierarchy);

template<size_t N> double bezier_max_deviation_to_quasi_interpolation(const std::vector<v<N>> &points)
{
    double z_coeff = bezier_z_coefficient(points.size() - 1);
    
    std::vector<v<N>> second_differences = centred_second_order_differences(points);
    
    double max_abs = 0;
    for (auto c: second_differences) {
        if (max_abs < l_inf(c))
        {
            max_abs = l_inf(c);
        }
    }
    
    return max_abs * z_coeff;
}

template<typename C> double bezier_max_deviation_to_quasi_interpolation_vec(const C &points)
{
    double z_coeff = bezier_z_coefficient(points.size() - 1);
    
    double max_abs = 0;
    
    for (int i = 1; i < points.size() - 1; i++)
    {
        auto p = (points[i-1] -2*points[i] + points[i+1]);
        double pl_inf = l_inf(p);
        if (max_abs < pl_inf)
        {
            max_abs = pl_inf;
        }
    }
    
    return max_abs * z_coeff;
}

template<size_t N> double bezier_max_deviation_to_quasi_interpolation(const varmesh<N> &m)
{
    double max_dev_rows = 0;
    for (int i = 0; i < m.row_size(); i++)
    {
        auto cur_row = row_ref_const<N>(m, i);
        double crl = bezier_max_deviation_to_quasi_interpolation_vec(cur_row);
        if (max_dev_rows < crl)
        {
            max_dev_rows = crl;
        }
    }
    
    double max_dev_cols = 0;
    
    for (int i = 0; i < m.col_size(); i++)
    {
        auto cur_col = col_ref_const<N>(m, i);
        double ccl = bezier_max_deviation_to_quasi_interpolation_vec(cur_col);
        if (max_dev_cols < ccl)
        {
            max_dev_cols = ccl;
        }
    }
    
    return max_dev_rows + max_dev_cols;
}

std::vector<double> bezier_quasi_interpolation_clipping(const std::vector<v<1>> points, double epsilon);
std::vector<double> get_intersections_quasi(const v2 &origin, const v2 &direction, const std::vector<v2> &points, double epsilon);
std::vector<double> get_intersections_quasi(const v2 &origin, const v2 &direction, const std::vector<v3> &points, double epsilon);


#endif /* intersection_hpp */
