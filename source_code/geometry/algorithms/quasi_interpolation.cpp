#include "quasi_interpolation.h"

#include <geometry/types/bezier_curve.h>
#include <geometry/types/bezier_surface.h>
#include <geometry/algorithms/bilinear_patch_intersection.h>

#include <iostream>
#include <algorithm>

std::vector<double> crossing(double a, double b)
{
    std::vector<double> crossings;

    if (a * b < 0)
    {
        crossings.push_back(a / (a - b));
    }

    if (a == 0)
    {
        crossings.push_back(0);
    }

    if (b == 0)
    {
        crossings.push_back(1);
    }

    return crossings;
}

v2 new_window(double e00, double e01, double e10, double e11)
{
    std::vector<double> crossings;

    if (e00 * e10 <= 0)
    {
        crossings.push_back(0);
    }

    if (std::fabs(e00) < 1E-13 || std::fabs(e10) < 1E-9)
    {
        crossings.push_back(0);
    }

    if (std::fabs(e10) < 1E-13 || std::fabs(e11) < 1E-9)
    {
        crossings.push_back(1);
    }

    if (e01 * e11 <= 0)
    {
        crossings.push_back(1);
    }

    auto additional = crossing(e00, e01);
    crossings.insert(crossings.end(), additional.begin(), additional.end());
    additional = crossing(e00, e11);
    crossings.insert(crossings.end(), additional.begin(), additional.end());
    additional = crossing(e10, e01);
    crossings.insert(crossings.end(), additional.begin(), additional.end());
    additional = crossing(e10, e11);
    crossings.insert(crossings.end(), additional.begin(), additional.end());

    if (crossings.size() > 0)
    {
        return v2{*std::min_element(crossings.begin(), crossings.end()), *std::max_element(crossings.begin(), crossings.end()) };
    }

    return v2{ 1, 0 };
}

v2 new_window_u(varmesh<2> mesh, int index)
{
    auto u0_window = new_window(mesh.element(0, 0)[0], mesh.element(0, 1)[0], mesh.element(1, 0)[0], mesh.element(1, 1)[0]);
    auto u1_window = new_window(mesh.element(0, 0)[1], mesh.element(0, 1)[1], mesh.element(1, 0)[1], mesh.element(1, 1)[1]);

    if (u0_window[1] - u0_window[0] >= 0 && u1_window[1] - u1_window[0] >= 0)
    {
        return v2{std::max(u0_window[0], u1_window[0]), std::min(u0_window[1], u1_window[1]) };
    }

    return v2{ 1, 0 };
}

v2 new_window_v(varmesh<2> mesh, int index)
{
    auto v0_window = new_window(mesh.element(0, 0)[0], mesh.element(1, 0)[0], mesh.element(0, 1)[0], mesh.element(1, 1)[0]);
    auto v1_window = new_window(mesh.element(0, 0)[1], mesh.element(1, 0)[1], mesh.element(0, 1)[1], mesh.element(1, 1)[1]);

    if (v0_window[1] - v0_window[0] >= 0 && v1_window[1] - v1_window[0] >= 0)
    {
        return v2{ std::max(v0_window[0], v1_window[0]), std::min(v0_window[1], v1_window[1]) };
    }

    return v2{ 1, 0 };
}

std::vector<v2> bilinear_patch_roots_clipping(varmesh<2> mesh, double epsilon)
{
    std::vector<v2> roots;

    int iteration = 0;

    std::deque<std::pair<v2, v2>> q;

    q.push_back({ {0, 1}, {0, 1} });

    while (!q.empty())
    {
        if (iteration > 1000)
        {
            break;
        }
        auto cw = q.front();

        q.pop_front();

        auto cm{ mesh };
        bezier_clip_surface(cm, cw.first, cw.second);

        auto nw_u = new_window_u(cm, 0);
        auto nw_v = new_window_v(cm, 1);

        double wd_u = nw_u[1] - nw_u[0];
        double wd_v = nw_v[1] - nw_v[0];

        if (0 <= wd_u && 0 <= wd_v)
        {
            auto new_window_u = v2{ cw.first[0] * (1 - nw_u[0]) + cw.first[1] * nw_u[0], cw.first[0] * (1 - nw_u[1]) + cw.first[1] * nw_u[1] };
            auto new_window_v = v2{ cw.second[0] * (1 - nw_v[0]) + cw.second[1] * nw_v[0], cw.second[0] * (1 - nw_v[1]) + cw.second[1] * nw_v[1] };

            auto result = std::make_pair(new_window_u, new_window_v);

            if (std::max(new_window_u[1] - new_window_u[0], new_window_v[1] - new_window_v[0]) < epsilon)
            {
                roots.push_back(v2{ v2{0.5, 0.5} *result.first , v2{ 0.5, 0.5 } *result.second});
            }
            else
            {
                if (wd_u < 0.75 && wd_v < 0.75)
                {
                    q.push_back(result);
                }
                else
                {
                    double mid_u = v2{ 0.5, 0.5 } *result.first;
                    double mid_v = v2{ 0.5, 0.5 } *result.second;

                    q.push_back(std::make_pair(v2{ result.first[0], mid_u }, v2{ result.second[0], mid_v }));
                    q.push_back(std::make_pair(v2{ mid_u, result.first[1] }, v2{ result.second[0], mid_v }));
                    q.push_back(std::make_pair(v2{ result.first[0], mid_u }, v2{ mid_v, result.second[1]}));
                    q.push_back(std::make_pair(v2{ mid_u, result.first[1] }, v2{ mid_v, result.second[1]}));
                }
            }
        }        

        iteration++;
    }

    return roots;
}

bool intervalls_overlap(const v2& i, const v2& j)
{
    if (i[0] < j[0] && i[0] < j[1] && i[1] < j[0] && i[1] < j[1])
    {
        return false;
    }
    
    if (i[0] > j[0] && i[0] > j[1] && i[1] > j[0] && i[1] > j[1])
    {
        return false;
    }
    
    return true;
}

std::vector<double> bezier_quasi_interpolation_clipping(const std::vector<v<1>> &points, double epsilon)
{
    std::vector<double> intersections;
    
    std::deque<v2> q;
    
    q.push_back(v2{{0, 1}});
    
    while(!q.empty())
    {
        v2 window = q.front();
        q.pop_front();
        
        auto clipped_mesh = points;
        
        if (1 > window[1])
        {
            clipped_mesh = subdivide_bezier_curve(clipped_mesh, window[1]).first;
        }
        
        if (0 < window[0])
        {
            clipped_mesh = subdivide_bezier_curve(clipped_mesh, window[0]/window[1]).second;
        }
        
        double max_deviation = bezier_max_deviation_to_quasi_interpolation(clipped_mesh);
        v2 cylinder{{-max_deviation, max_deviation}};
        
        auto quasi = bezier_curve_quasi_interpolation(clipped_mesh);
        
        for (int i = 0; i < quasi.size() - 1; i++)
        {
            if (intervalls_overlap(cylinder, v2{{quasi[i][0], quasi[i+1][0]}})) {
                if (max_deviation < epsilon)
                {
                    if (quasi[i][0]*quasi[i+1][0] < 0)
                    {
                        double ratio = quasi[i][0]/(quasi[i+1][0] - quasi[i][0]);
                        double crossing = window[0] + (i + fabs(ratio))/(quasi.size() - 1) * (window[1] - window[0]);
                        intersections.push_back(crossing);
                    }
                    else
                    {
                        if (quasi[i][0] == 0)
                        {
                            intersections.push_back(quasi[i][0]);
                        }
                        if (quasi[i+1][0] == 0)
                        {
                            intersections.push_back(quasi[i+1][0]);
                        }
                    }
                }
                else
                {
                    double diff = (window[1] - window[0])/(quasi.size() - 1);
                    v2 sub_intervall{{diff * i + window[0], diff * (i+1) + window[0]}};
                    q.push_back(sub_intervall);
                }
            }
        }
    }
    
    return intersections;
}

std::vector<double> get_intersections_quasi(const v2 &origin, const v2 &direction, const std::vector<v2> &points, double epsilon)
{
    v3 hesse = hesse_from(origin, origin + direction);
    
    std::vector<v1> projected_points;
    
    for (int i = 0; i < points.size(); i++)
    {
        projected_points.push_back(v1{{add_dimension(points[i]) * hesse}});
    }
    
    return bezier_quasi_interpolation_clipping(projected_points, epsilon);
}

std::vector<double> get_intersections_quasi(const v2 &origin, const v2 &direction, const std::vector<v3> &points, double epsilon)
{
    v3 hesse = hesse_from(origin, origin + direction);
    
    std::vector<v1> projected_points;
    
    for (int i = 0; i < points.size(); i++)
    {
        projected_points.push_back(v1{{points[i] * hesse}});
    }
    
    return bezier_quasi_interpolation_clipping(projected_points, epsilon);
}

std::vector<std::vector<bool>> does_increased_mesh_contain_origin(const varmesh<2>& mesh, double offset)
{
    std::vector<std::vector<bool>> overlaps(mesh.row_size() - 1, std::vector<bool>(mesh.col_size() - 1, false));
    std::vector<std::vector<double>> dist2_to_origin(mesh.row_size(), std::vector<double>(mesh.col_size(), 0));

    double offset2 = offset * offset;

    for (int i = 0; i < mesh.row_size(); i++)
    {
        for (int j = 0; j < mesh.col_size(); j++)
        {
            dist2_to_origin[i][j] = length2(mesh.element(i, j));
        }
    }

    for (int i = 0; i < mesh.row_size(); i++)
    {
        for (int j = 0; j < mesh.col_size() - 1; j++)
        {
            bool cur_overlaps = dist2_to_origin[i][j] <= offset2 || dist2_to_origin[i][j + 1] <= offset2;

            if (!cur_overlaps)
            {
                const v2& p = mesh.element(i, j);
                const v2& q = mesh.element(i, j + 1);
                v3 hesse = unnormalized_hesse_from(p, q);

                double d = hesse[2];

                v2 n{ {hesse[0], hesse[1]} };
                double nlen2 = length2(n);

                if (0 < nlen2)
                {
                    v2 closest_point_of_line = (-d / nlen2) * n;

                    v3 ortho_hesse = unnormalized_hesse_from(v2{ {0, 0} }, closest_point_of_line);

                    if (0 > ortho_hesse * add_dimension(p) * ortho_hesse * add_dimension(q))
                    {
                        cur_overlaps = length2(closest_point_of_line) <= offset2;
                    }
                }

            }

            if (cur_overlaps)
            {
                if (0 < i)
                {
                    overlaps[i - 1][j] = true;
                }
                if (i < mesh.row_size() - 1)
                {
                    overlaps[i][j] = true;
                }
            }
        }
    }
    for (int i = 0; i < mesh.row_size() - 1; i++)
    {
        for (int j = 0; j < mesh.col_size(); j++)
        {
            bool cur_overlaps = dist2_to_origin[i][j] <= offset2 || dist2_to_origin[i + 1][j] <= offset2;

            if (!cur_overlaps)
            {
                const v2& p = mesh.element(i, j);
                const v2& q = mesh.element(i + 1, j);
                v3 hesse = unnormalized_hesse_from(p, q);

                double d = hesse[2];

                v2 n{ {hesse[0], hesse[1]} };
                double nlen2 = length2(n);

                if (0 < nlen2)
                {
                    v2 closest_point_of_line = (-d / nlen2) * n;

                    v3 ortho_hesse = unnormalized_hesse_from(v2{ {0, 0} }, closest_point_of_line);

                    if (0 > ortho_hesse * add_dimension(p) * ortho_hesse * add_dimension(q))
                    {
                        cur_overlaps = length2(closest_point_of_line) <= offset2;
                    }
                }

            }
            if (cur_overlaps)
            {
                if (0 < j)
                {
                    overlaps[i][j - 1] = true;
                }
                if (j < mesh.col_size() - 1)
                {
                    overlaps[i][j] = true;
                }
            }
        }
    }

    for (int i = 0; i < mesh.row_size() - 1; i++)
    {
        for (int j = 0; j < mesh.col_size() - 1; j++)
        {
            bool cur_overlap = overlaps[i][j];
            if (!cur_overlap)
            {
                std::vector<v2> quad(4, v2{ {0, 0} });
                quad[0] = (mesh.element(i, j));
                quad[1] = (mesh.element(i, j + 1));
                quad[2] = (mesh.element(i + 1, j + 1));
                quad[3] = (mesh.element(i + 1, j));

                cur_overlap = origin_inside_polygon(quad);

                overlaps[i][j] = cur_overlap;
            }
        }
    }

    return overlaps;
}

int signum(double d)
{
    return d > 0 ? 1 : (d < 0 ? -1 : 0);
}

std::vector<v2> bezier_quasi_interpolation_clipping(const varmesh<2> &points, double epsilon)
{
    if (points.col_size() == 2 && points.row_size() == 2)
    {
        return bilinear_patch_roots_clipping(points, epsilon); // 
    }
    std::vector<v2> intersections;
    
    std::deque<std::pair<v2, v2>> q;
    
    q.push_back(std::make_pair(v2{{0, 1}}, v2{{0, 1}}));
    
    varmesh<2> clipped_mesh(points.row_size(), points.col_size());

    int iteration_count = 0;
    
    while(!q.empty())
    {
        iteration_count++;

        auto windows = q.front();
        q.pop_front();
        
        clipped_mesh = points;
        
        bezier_clip_surface(clipped_mesh, windows.first, windows.second);
        
        double max_deviation = bezier_max_deviation_to_quasi_interpolation(clipped_mesh);
        
        double max_deviation2 = max_deviation * max_deviation;
        
        auto quasi = bezier_surface_quasi_interpolation(clipped_mesh);
        
        double window_diff_u = windows.first[1] - windows.first[0];
        double window_diff_v = windows.second[1] - windows.second[0];

        if (max_deviation < epsilon && window_diff_u < epsilon && window_diff_v < epsilon)
        {
            double u = (windows.first[0] + windows.first[1]) / 2;
            double v = (windows.second[0] + windows.second[1]) / 2;
            intersections.push_back(v2{ {u, v} });
        }
        else
        {             
            auto does_contain_origin = does_increased_mesh_contain_origin(quasi, max_deviation);

            for (int i = 0; i < points.row_size() - 1; i++)
            {
                for (int j = 0; j < points.col_size() - 1; j++)
                {
                    if (does_contain_origin[i][j])
                    {
                        double diffu = (windows.first[1] - windows.first[0]) / (points.col_size() - 1);
                        v2 sub_intervall_u{ {diffu * j + windows.first[0], diffu * (j + 1) + windows.first[0]} };
                        double diffv = (windows.second[1] - windows.second[0]) / (points.row_size() - 1);
                        v2 sub_intervall_v{ {diffv * i + windows.second[0], diffv * (i + 1) + windows.second[0]} };
                        q.push_back(std::make_pair(sub_intervall_u, sub_intervall_v));
                    }
                }
            }
        }        
    }

    return intersections;
}

std::vector<v2> get_intersections_quasi(const v3& origin, const v3& direction, const varmesh<4>& m, double epsilon)
{
    v4 horizontal_plane = horizontal_plane_of_ray(origin, direction);
    v4 vertical_plane = vertical_plane_of_ray(origin, direction);

    auto pm = project_mesh(m, vertical_plane, horizontal_plane);

    return bezier_quasi_interpolation_clipping(pm, epsilon);
}

