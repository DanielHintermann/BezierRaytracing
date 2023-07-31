#include "quasi_interpolation.h"

#include <geometry/types/bezier_curve.h>
#include <geometry/types/bezier_surface.h>

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

std::vector<double> bezier_quasi_interpolation_clipping(const std::vector<v<1>> points, double epsilon)
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
            dist2_to_origin[i][j] = length2(mesh[i][j]);
        }
    }

    for (int i = 0; i < mesh.row_size(); i++)
    {
        for (int j = 0; j < mesh.col_size() - 1; j++)
        {
            bool cur_overlaps = dist2_to_origin[i][j] <= offset2 || dist2_to_origin[i][j + 1] <= offset2;

            if (!cur_overlaps)
            {
                const v2& p = mesh[i][j];
                const v2& q = mesh[i][j + 1];
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
                const v2& p = mesh[i][j];
                const v2& q = mesh[i + 1][j];
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
                quad[0] = (mesh[i][j]);
                quad[1] = (mesh[i][j + 1]);
                quad[2] = (mesh[i + 1][j + 1]);
                quad[3] = (mesh[i + 1][j]);

                cur_overlap = origin_inside_polygon(quad);

                overlaps[i][j] = cur_overlap;
            }
        }
    }

    return overlaps;
}

std::vector<v2> bezier_quasi_interpolation_clipping(const varmesh<2> &points, double epsilon)
{
    std::vector<v2> intersections;
    
    std::deque<std::pair<v2, v2>> q;
    
    q.push_back(std::make_pair(v2{{0, 1}}, v2{{0, 1}}));
    
    varmesh<2> clipped_mesh(points.row_size(), points.col_size());
    
    while(!q.empty())
    {
        auto windows = q.front();
        q.pop_front();
        
        clipped_mesh = points;
        
        bezier_clip_surface(clipped_mesh, windows.first, windows.second);
        
        double max_deviation = bezier_max_deviation_to_quasi_interpolation(clipped_mesh);
        double max_deviation2 = max_deviation * max_deviation;
        
        auto quasi = bezier_surface_quasi_interpolation(clipped_mesh);
        
        
        if (max_deviation < epsilon)
        {
            varmesh<2> m(2, 2);
            m[0][0] = quasi[0][0];
            m[0][1] = quasi[0][points.col_size()-1];
            m[1][0] = quasi[points.row_size()-1][0];
            m[1][1] = quasi[points.row_size()-1][points.col_size()-1];
            
            auto solutions = solve_linear_form(v2{{0,0}}, m, epsilon);
            for (int sol = 0; sol < solutions.size(); sol++)
            {
                double diffu = windows.first[1] - windows.first[0];
                double diffv = windows.second[1] - windows.second[0];
                double u = windows.first[0] + solutions[sol][0] * diffu;
                double v = windows.second[0] + solutions[sol][1] * diffv;                        
                        
                intersections.push_back(v2{{u, v}});
            }
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

