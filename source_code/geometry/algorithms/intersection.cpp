#include <cfloat>
#include <algorithm>
#include <iostream>

#include "intersection.h"

double intersection_with_triangle(const v3 &origin, const v3 &direction, const v3 &u, const v3 &v, const v3 &w)
{
    v3 e1 = v - u;
    v3 e2 = w - u;
    
    v3 pvec = cross_product(direction, e2);
    
    double det = e1 * pvec;
    
    if (DBL_EPSILON > std::abs(det))
    {
        return std::numeric_limits<double>::max();
    }
    
    v3 tvec = origin - u;
    
    double uscale = (tvec * pvec)/det;
    
    if (0 > uscale || 1 < uscale)
    {
        return std::numeric_limits<double>::max();
    }
    
    v3 qvec = cross_product(tvec, e1);
    
    double vscale = (direction * qvec)/det;
    
    if (0 > vscale || 1 < vscale + uscale)
    {
        return std::numeric_limits<double>::max();
    }
    
    return (e2 * qvec)/det;
}

int sign(double v)
{
    if (0 < v)
        return 1;
    if (0 > v)
        return -1;
    return 0;
}

double angle(const v2 &v)
{
    double angle = atan2(v[0], v[1]);
    
    if (0 > angle)
    {
        angle = 2 * std::numbers::pi + angle;
    }
    
    return angle;
}

bool intersects_convex_hull(const v3 &origin, const v3 &dir, const std::vector<v3> &points)
{
    std::vector<v2> projection_coordinates;
    
    v4 horizontal_plane = horizontal_plane_of_ray(origin, dir);
    v4 vertical_plane = vertical_plane_of_ray(origin, dir);
    
    for (auto point = points.begin(); point != points.end(); point++)
    {
        v4 point_with_added_dim = add_dimension(*point);
        v2 projection_coordinated_point{{horizontal_plane * point_with_added_dim, vertical_plane * point_with_added_dim}};
        projection_coordinates.push_back(projection_coordinated_point);
        
        if (projection_coordinated_point == v2{{0, 0}})
            return true;
    }
    
    //bool res_atan2 = is_origin_in_convex_hull_atan2(projection_coordinates);
    bool res = is_origin_in_convex_hull(projection_coordinates);
    
    //if (res_atan2 != res)
    //{
    //    std::cout << "\nOh no...";
    //    res = is_origin_in_convex_hull(projection_coordinates);
    //}
    
    return res;
}

bool is_origin_in_convex_hull_atan2(std::vector<v2> points)
{
    std::vector<double> angles;
    for (auto point = points.begin(); point != points.end(); point++)
    {
        angles.push_back(angle(*point));
    }
    
    std::sort(angles.begin(), angles.end());
    
    
    if (std::numbers::pi < (2 * std::numbers::pi + angles[0]) - angles[angles.size() - 1])
    {
        return false;
    }
    
    for (int i = 1; i < angles.size(); i++)
    {
        if (std::numbers::pi < angles[i] - angles[i - 1])
        {
            return false;
        }
    }
    
    return true;
}

bool is_origin_in_convex_hull(std::vector<v2> points)
{
    std::sort(points.begin(), points.end(), compare_polar);
    
    if (!counter_clockwise_angle_not_greater_180(points[points.size() - 1], points[0]))
    {
        return false;
    }
    
    for (int i = 1; i < points.size(); i++)
    {
        if (!counter_clockwise_angle_not_greater_180(points[i - 1], points[i]))
        {
            return false;
        }
    }
    
    return true;
}

bool intersects_e1_halfline(const v2& p, const v2 q)
{
    if (0 < p[1]*q[1])
    {
        return false;
    }
    if (0 == p[1]*q[1])
    {
        // only report intersection in case of hitting the vertex, if the other vertex is below e1 half line
        return (0 == p[1] && 0 <= p[0] && 0 > q[1]) || (0 == q[1] && 0 <= q[0] && 0 > p[1]);
    }
    double ratio = p[1] / (p[1] - q[1]);
    double intersection = (1 - ratio) * p[0] + ratio * q[0];
    
    return 0 <= intersection && 0 <= ratio && 1 >= ratio;
}

bool origin_inside_polygon(const std::vector<v2> &polygon)
{
    int intersections = 0;
    
    for (int i = 0; i < polygon.size(); i++)
    {
        if (intersects_e1_halfline(polygon[i], polygon[(i + 1)%polygon.size()]))
        {
            intersections++;
        }
    }
    
    return 1 == intersections % 2;
}

bool inside_polygon(const v2& point, const std::vector<v2> &polygon)
{
    std::vector<v2> translated;
    for (int i = 0; i < polygon.size(); i++)
    {
        translated.push_back(polygon[i] - point);
    }
    
    return origin_inside_polygon(translated);
}

v2 closest_point_to_origin_of_segment(const v2& p, const v2& q)
{
    v3 hesse = unnormalized_hesse_from(p, q);
    
    double d = v3{{0, 0, 1}} * hesse;
    
    v2 n{{hesse[0], hesse[1]}};
    double nlen2 = length2(n);
    
    if (0 < nlen2)
    {
    v2 closest_point_of_line = (-d/nlen2) * n;
    
    v3 ortho_hesse = unnormalized_hesse_from(v2{{0, 0}}, closest_point_of_line);
    
    if (0 > ortho_hesse * add_dimension(p) * ortho_hesse * add_dimension(q))
    {
        return closest_point_of_line;
    }
    }
    
    if (length2(p) < length2(q))
    {
        return p;
    }
    else
    {
        return q;
    }
}

bool origin_circle_overlaps_segment(const v2& p, const v2& q, double radius2)
{
    if (length2(p) <= radius2 || length2(q) <= radius2)
    {
        return true;
    }
    v3 hesse = unnormalized_hesse_from(p, q);
    
    double d = v3{{0, 0, 1}} * hesse;
    
    v2 n{{hesse[0], hesse[1]}};
    double nlen2 = length2(n);
    
    if (0 < nlen2)
    {
        v2 closest_point_of_line = (-d/nlen2) * n;
        
        v3 ortho_hesse = unnormalized_hesse_from(v2{{0, 0}}, closest_point_of_line);
        
        if (0 > ortho_hesse * add_dimension(p) * ortho_hesse * add_dimension(q))
        {
            return length2(closest_point_of_line) <= radius2;
        }
    }
    
    return false;
}

bool circle_overlaps_polygon(double radius, const std::vector<v<2>> &polygon)
{
    double r2 = radius * radius;
    
    for (int i = 0; i < polygon.size(); i++)
    {
        v2 closest_point = closest_point_to_origin_of_segment(polygon[i], polygon[(i + 1)%polygon.size()]);
        if (length2(closest_point) <= r2)
        {
            return true;
        }
    }
    
    return origin_inside_polygon(polygon);
}

std::vector<v3> get_points(const varmesh<4> &m)
{
    std::vector<v3> result;

    for (int i = 0; i < m.row_size(); i++)
    {
        for (int j = 0; j < m.col_size(); j++)
        {
            result.push_back(remove_dimension(m.element(i, j)));
        }
    }

    return result;
}

std::vector<int> intersects_mesh(const v3& origin, const v3& ray, const std::vector<varmesh<4>>& meshes_hierarchy)
{
    std::vector<int> intersected_meshes;

    std::deque<size_t> mesh_index_to_intersect;
    mesh_index_to_intersect.push_back(0);

    while (!mesh_index_to_intersect.empty())
    {
        size_t index = mesh_index_to_intersect.front();
        mesh_index_to_intersect.pop_front();
        if (intersects_convex_hull(origin, ray, get_points(meshes_hierarchy[index])))
        {
            size_t new_index = 4 * index + 1;
            if (new_index < meshes_hierarchy.size())
            {
                for (int i = 0; i < 4; i++)
                {
                    mesh_index_to_intersect.push_back(new_index + i);
                }
            }
            else
            {
                intersected_meshes.push_back(index);
            }
        }
    }

    return intersected_meshes;
}

