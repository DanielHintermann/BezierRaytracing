#include <gtest/gtest.h>

#include <raytracing/nurbs_raytracing.h>
#include <geometry/types/bezier_curve.h>
#include <geometry/types/bezier_surface.h>

using ::testing::InitGoogleTest;
using ::testing::Test;
using ::testing::TestCase;
using ::testing::TestEventListeners;
using ::testing::TestInfo;
using ::testing::TestPartResult;
using ::testing::UnitTest;

template<size_t N> void expect_near(v<N> u, v<N> w)
{
    for (int i = 0; i < N; i++)
    {
        EXPECT_NEAR(u[i], w[i], 1e-06);
    }
}

TEST(Nurbs, evaluateBezier2)
{
    std::vector<v2> controlPoints{v2{{1, 2}}, v2{{3, 8}}};

	v2 expected{{2,5}};

	EXPECT_EQ(expected, evaluate_bezier_curve(controlPoints, 0.5));
}

TEST(Nurbs, evaluateBezier3)
{
    std::vector<v2> controlPoints{v2{{0, 1}}, v2{{2, 3}}, v2{{4, 13}}};

	v2 expected{{2, 5}};

	EXPECT_EQ(expected, evaluate_bezier_curve(controlPoints, 0.5));
}

TEST(Nurbs, evaluateBezierSurface)
{
    varmesh<2> m(2, 2);
    m.element(0, 0) = v2{ 1, -1 };
    m.element(0, 1) = v2{ 2, -2 };
    m.element(1, 0) = v2{ 3, -3 };
    m.element(1, 1) = v2{ 4, -4 };

    auto y = evaluate_bezier_surface(m, 0, 0);

    EXPECT_EQ(y, (v2{ 1, -1 }));

    y = evaluate_bezier_surface(m, 1, 0);

    EXPECT_EQ(y, (v2{ 2, -2 }));

    y = evaluate_bezier_surface(m, 0, 1);

    EXPECT_EQ(y, (v2{ 3, -3 }));

    y = evaluate_bezier_surface(m, 1, 1);

    EXPECT_EQ(y, (v2{ 4, -4 }));
}

TEST(Nurbs, bezierSurfaceClipping)
{
    varmesh<2> m(2, 2);
    m.element(0, 0) = v2{ 1, -1 };
    m.element(0, 1) = v2{ 2, -2 };
    m.element(1, 0) = v2{ 3, -3 };
    m.element(1, 1) = v2{ 4, -4 };

    auto c{ m };
    bezier_clip_surface(c, { 0.9, 1 }, { 0, 1 });

    EXPECT_EQ(c.element(0, 1), (v2{ 2, -2 }));
    EXPECT_EQ(c.element(1, 1), (v2{ 4, -4 }));
    EXPECT_EQ(c.element(0, 0), (v2{ 1.9, -1.9 }));
    EXPECT_EQ(c.element(1, 0), (v2{ 3.9, -3.9 }));
}

TEST(Nurbs, test_subdivide_bezier)
{
    v2 k{{1, 2}};
    v2 l{{4, 7}};
    v2 m{{2, 3}};

	std::vector<v2> given{k, l, m};

    v2 expected1{{2.5, 4.5}};
    v2 expected2{{3, 5}};
    v2 expected12{{2.75, 4.75}};
	std::vector<v2> expected_left{k, expected1, expected12};
	std::vector<v2> expected_right{expected12, expected2, m};

	EXPECT_EQ(std::make_pair(expected_left, expected_right), subdivide_bezier_curve(given, 0.5));
}

TEST(Nurbs, test_intersect_with_triangle)
{
    v3 u{{2, 2, 2}};
    v3 v{{4, 4, 2}};
    v3 w{{2, 8, 2}};

    v3 o{{0, 0, 0}};

    EXPECT_EQ(2, intersection_with_triangle(o, v3{{1, 1, 1}}, u, v, w));
    EXPECT_EQ(2, intersection_with_triangle(o + v3{{2,3,5}}, v3{{1, 1, 1}}, u + v3{{2,3,5}}, v + v3{{2,3,5}}, w + v3{{2,3,5}}));

    EXPECT_EQ(2, intersection_with_triangle(o, v3{{2, 2, 1}}, u, v, w));
    EXPECT_EQ(2, intersection_with_triangle(o, v3{{1, 4, 1}}, u, v, w));
    EXPECT_EQ(1, intersection_with_triangle(o, v3{{3, 5, 2}}, u, v, w));
}

TEST(Nurbs, test_rotate_around_x)
{
    v3 e1{{1, 0, 0}};
    v3 e2{{0, 1, 0}};
    v3 e3{{0, 0, 1}};
    
    expect_near(e1, remove_dimension(rotation_in_e1(std::numbers::pi * 0.5) * add_dimension(e1)));
    expect_near(e1, remove_dimension(rotation_in_e1(std::numbers::pi * -0.5) * add_dimension(e1)));
    
    expect_near(e3, remove_dimension(rotation_in_e1(std::numbers::pi * 0.5) * add_dimension(e2)));
    expect_near(-e3, remove_dimension(rotation_in_e1(std::numbers::pi * -0.5) * add_dimension(e2)));
    
    expect_near(-e2, remove_dimension(rotation_in_e1(std::numbers::pi * 0.5) * add_dimension(e3)));
    expect_near(e2, remove_dimension(rotation_in_e1(std::numbers::pi * -0.5) * add_dimension(e3)));
}

TEST(Nurbs, test_intersects_convex_hull)
{
    std::vector<v3> points;
    points.push_back(v3{{1, 1, 2}});
    points.push_back(v3{{4, 1, 2}});
    points.push_back(v3{{1, 4, 2}});
    
    v3 origin{{0,0,0}};
    
    v3 dir{{2, 2, 2}};
    
    EXPECT_TRUE(intersects_convex_hull(origin, dir, points));
}

TEST(Nurbs, test_vertical_plane_of_ray)
{
    /*
     p = (1, 1, 1)
     q = (2, 100, 1)
     
     hesse = get_vertical_plane(p, q)
     
     self.assertEqual((0, 0, -1, 1), hesse)
     
     self.assertEqual(0, dot(as_projective_point(p), hesse))
     self.assertEqual(0, dot(as_projective_point(q), hesse))
     
     self.assertTrue(0 < dot(as_projective_point((2, 1, 0)), hesse))
     */
    
    v3 p{{1, 1, 1}};
    v3 q{{3, 200, 1}};
    
    std::array<double, 4> expected{{0,0,-1,1}};
    
    EXPECT_EQ(expected, vertical_plane_of_ray(p, q - p));
    
    v3 r{{2, 1, 0}};
    EXPECT_EQ(1, add_dimension(r) * vertical_plane_of_ray(p, q - p));
    
    r = v3{{2, 1, 2}};
    EXPECT_EQ(-1, add_dimension(r) * vertical_plane_of_ray(p, q - p));
}

TEST(Nurbs, test_quasi_get_intersections)
{
    auto points = std::vector<v<1>>{{
        v1{{-1}}, v1{{4}}, v1{{-1}}
    }};
    
    auto result = bezier_quasi_interpolation_clipping(points, 1e-6);
    
    ASSERT_EQ(2, result.size());
    
    auto y1 = evaluate_bezier_curve(points, result[0]);
    
    EXPECT_NEAR(0, y1[0], 1e-6);
    
    auto y2 = evaluate_bezier_curve(points, result[1]);
    
    EXPECT_NEAR(0, y2[0], 1e-6);
}

TEST(Nurbs, test_bezier_max_deviation_to_quasi_interpolation)
{
    auto points = std::vector<v<1>>{{
        v1{{-1}}, v1{{4}}, v1{{-1}}
    }};
    
    auto quasi_interpolating_polygon = bezier_curve_quasi_interpolation(points);
    
    double max_deviation = bezier_max_deviation_to_quasi_interpolation(points);
    
    for (double u = 0; u <= 1; u += 0.001)
    {
        double diff = evaluate_polyline(quasi_interpolating_polygon, u)[0] - evaluate_bezier_curve(points, u)[0];
        
        EXPECT_LE(abs(diff), max_deviation + 1E-12);
    }
}

TEST(Nurbs, test_bezier_max_deviation_to_quasi_interpolation0p25)
{
    auto points = std::vector<v<1>>{{
        v1{{-1}}, v1{{4}}, v1{{-1}}
    }};
    
    auto quasi_interpolating_polygon = bezier_curve_quasi_interpolation(points);
    
    double max_deviation = bezier_max_deviation_to_quasi_interpolation(points);
    
    double q = evaluate_polyline(quasi_interpolating_polygon, 0.25)[0];
    double p = evaluate_bezier_curve(points, 0.25)[0];
    
    double diff =  q - p;
        
    EXPECT_LE(abs(diff), max_deviation);
}

TEST(Nurbs, test_invBilinear)
{
    v2 a{{1, 2}}; v2 e00{{0, 0}};
    v2 b{{3, 0}}; v2 e10{{1, 0}};
    v2 c{{4, 8}}; v2 e11{{1, 1}};
    v2 d{{0, 9}}; v2 e01{{0, 1}};
    
    EXPECT_EQ(e00, invBilinear(a, a, b, c, d));
    EXPECT_EQ(e10, invBilinear(b, a, b, c, d));
    EXPECT_EQ(e11, invBilinear(c, a, b, c, d));
    EXPECT_EQ(e01, invBilinear(d, a, b, c, d));
    
    v2 expected{{0.5, 0}}; v2 p{{2, 1}};
    EXPECT_EQ(expected, invBilinear(p, a, b, c, d));
    
    expected = v2{{0, 0.5}}; p = v2{{0.5, 5.5}};
    EXPECT_EQ(expected, invBilinear(p, a, b, c, d));
    
    expected = v2{{0.5, 1}}; p = v2{{2, 8.5}};
    EXPECT_EQ(expected, invBilinear(p, a, b, c, d));
    
    expected = v2{{1, 0.5}}; p = v2{{3.5, 4}};
    EXPECT_EQ(expected, invBilinear(p, a, b, c, d));
    
    expected = v2{{0.5, 0.5}}; p = v2{{2, 9.5/2}};
    EXPECT_EQ(expected, invBilinear(p, a, b, c, d));
}

TEST(Nurbs, test_invBilinear_bug)
{
    v2 d{{-1, -1}}; v2 e00{{0, 0}};
    v2 c{{0, -1}}; v2 e10{{1, 0}};
    v2 b{{0, 0}}; v2 e11{{1, 1}};
    v2 a{{-1, 0}}; v2 e01{{0, 1}};
    
    auto solutions = solve_linear_form(v2{{-0.5,-0.5}}, a, b, c, d);
    
    EXPECT_EQ(e00, invBilinear(a, a, b, c, d));
    EXPECT_EQ(e10, invBilinear(b, a, b, c, d));
    EXPECT_EQ(e11, invBilinear(c, a, b, c, d));
    EXPECT_EQ(e01, invBilinear(d, a, b, c, d));
}

TEST(Nurbs, test_solve_linear_form_range)
{
    varmesh<2> m(2, 2);
    m[0][0] = v2{{-2, -1}};
    m[0][1] = v2{{0, -1}};
    m[1][0] = v2{{-1, 0}};
    m[1][1] = v2{{0, 0}};
    
    for (double uu = 0; uu <= 1; uu+= 0.01)
    {
        for (double vv = 0; vv <= 1; vv+= 0.01)
        {
            auto y = evaluate_bezier_surface(m, uu, vv);
            
            auto solution_v = solve_linear_form(y, m[0][0], m[1][0], m[0][1], m[1][1]);
            EXPECT_EQ(1, solution_v.size());
            
            auto solution_u = solve_linear_form(y, m[0][0], m[0][1], m[1][0], m[1][1]);
            EXPECT_EQ(1, solution_u.size());
            
            v2 expected{{uu, vv}};
            v2 solution{{solution_u[0], solution_v[0]}};
            expect_near(expected, solution);
        }
    }
}

TEST(Nurbs, test_solve_linear_form_range2)
{
    varmesh<2> m(2, 2);
    m[0][0] = v2{{-2, -1}};
    m[0][1] = v2{{0, -0.5}};
    m[1][0] = v2{{-4, 0.5}};
    m[1][1] = v2{{1, 0}};
    
    for (double uu = 0; uu <= 1; uu+= 0.01)
    {
        for (double vv = 0; vv <= 1; vv+= 0.01)
        {
            auto y = evaluate_bezier_surface(m, uu, vv);
            
            auto solutions = solve_linear_form(y, m, 1e-9);
            EXPECT_EQ(1, solutions.size());
            if (solutions.size() > 0)
            {
                v2 expected({ uu, vv });
                //expect_near(expected, solutions[0]);
            }
        }
    }
}

TEST(Nurbs, test_intersects_e1_halfline)
{
    EXPECT_TRUE(intersects_e1_halfline(v2{{0, -1}}, v2{{0, 1}}));
    EXPECT_TRUE(intersects_e1_halfline(v2{{1, -1}}, v2{{1, 1}}));
    EXPECT_FALSE(intersects_e1_halfline(v2{{-1, -1}}, v2{{-1, 1}}));
    EXPECT_FALSE(intersects_e1_halfline(v2{{1, -1}}, v2{{1, -0.000001}}));
    EXPECT_FALSE(intersects_e1_halfline(v2{{1, 1}}, v2{{1, 0.000001}}));
    EXPECT_TRUE(intersects_e1_halfline(v2{{-1, -1}}, v2{{1.01, 1}}));
    EXPECT_FALSE(intersects_e1_halfline(v2{{-1, -1}}, v2{{0.99, 1}}));
    EXPECT_TRUE(intersects_e1_halfline(v2{{-1, -1}}, v2{{0, 0}}));
    EXPECT_TRUE(intersects_e1_halfline(v2{{0, 0}}, v2{{-1, -1}}));
    EXPECT_FALSE(intersects_e1_halfline(v2{{1, 0}}, v2{{2, 0.001}}));
    EXPECT_FALSE(intersects_e1_halfline(v2{{2, 0.001}}, v2{{1, 0}}));
    EXPECT_TRUE(intersects_e1_halfline(v2{{2, -0.001}}, v2{{3, 0}}));
}

TEST(Nurbs, test_inside_polygon)
{
    std::vector<v2> points;
    
    points.push_back(v2{{1, 1}});
    points.push_back(v2{{2, 0}});
    points.push_back(v2{{2, 1}});
    points.push_back(v2{{3, 1}});
    points.push_back(v2{{3, 2}});
    points.push_back(v2{{1, 2}});
    
    EXPECT_TRUE(inside_polygon(v2{{1.5, 1}}, points));
    EXPECT_TRUE(inside_polygon(v2{{2, 1.2}}, points));
    EXPECT_FALSE(inside_polygon(v2{{0, 1}}, points));
    EXPECT_FALSE(inside_polygon(v2{{4, 1.2}}, points));
    EXPECT_FALSE(inside_polygon(v2{{1.5, 0.1}}, points));
}

TEST(Nurbs, test_closest_point_to_origin_of_segment)
{
    v2 expected{{1, 0}};
    
    expect_near(expected, closest_point_to_origin_of_segment(v2{{1, 0}}, v2{{2, 0}}));
    expect_near(expected, closest_point_to_origin_of_segment(v2{{2, 0}}, v2{{1, 0}}));
    expect_near(expected, closest_point_to_origin_of_segment(v2{{1, 0}}, v2{{1, 0}}));
    expected = v2{{1, 1}};
    expect_near(expected, closest_point_to_origin_of_segment(v2{{1, 1}}, v2{{2, 1}}));
    expect_near(expected, closest_point_to_origin_of_segment(v2{{2, 1}}, v2{{1, 1}}));
    expect_near(expected, closest_point_to_origin_of_segment(v2{{2, 0}}, v2{{0, 2}}));
    expect_near(expected, closest_point_to_origin_of_segment(v2{{0, 2}}, v2{{2, 0}}));
    expect_near(expected, closest_point_to_origin_of_segment(v2{{-1, 3}}, v2{{2, 0}}));
    expected = v2{{-1, -1}};
    expect_near(expected, closest_point_to_origin_of_segment(v2{{-1, -1}}, v2{{-2, -1}}));
    expect_near(expected, closest_point_to_origin_of_segment(v2{{-2, -1}}, v2{{-1, -1}}));
    expect_near(expected, closest_point_to_origin_of_segment(v2{{-2, 0}}, v2{{0, -2}}));
    expect_near(expected, closest_point_to_origin_of_segment(v2{{0, -2}}, v2{{-2, 0}}));
    expect_near(expected, closest_point_to_origin_of_segment(v2{{1, -3}}, v2{{-2, 0}}));
}

TEST(Nurbs, test_circle_overlaps_polygon)
{
    std::vector<v2> polygon;
    
    polygon.push_back(v2{{-2, 0}});
    polygon.push_back(v2{{1, 3}});
    
    EXPECT_FALSE(circle_overlaps_polygon(0, polygon));
    EXPECT_FALSE(circle_overlaps_polygon(1, polygon));
    EXPECT_FALSE(circle_overlaps_polygon(sqrt(2) - 0.1, polygon));
    EXPECT_TRUE(circle_overlaps_polygon(sqrt(2) + 0.1, polygon));
    EXPECT_TRUE(circle_overlaps_polygon(1000, polygon));
}

TEST(Nurbs, test_circle_overlaps_polygon_long_edge)
{
    std::vector<v2> polygon;
    
    polygon.push_back(v2{{-200000, -1}});
    polygon.push_back(v2{{200000, -1}});
    
    EXPECT_FALSE(circle_overlaps_polygon(0, polygon));
    EXPECT_FALSE(circle_overlaps_polygon(0.999, polygon));
    EXPECT_TRUE(circle_overlaps_polygon(1.001, polygon));
    EXPECT_TRUE(circle_overlaps_polygon(9999999999999, polygon));
}

TEST(Nurbs, test_circle_overlaps_polygon_inside_polygon)
{
    std::vector<v2> polygon;
    
    polygon.push_back(v2{{-2000, 2000}});
    polygon.push_back(v2{{2000, 2000}});
    polygon.push_back(v2{{2000, -2000}});
    polygon.push_back(v2{{-2000, -2000}});
    
    EXPECT_TRUE(circle_overlaps_polygon(0, polygon));
    EXPECT_TRUE(circle_overlaps_polygon(1, polygon));
    EXPECT_TRUE(circle_overlaps_polygon(1000000, polygon));
}

TEST(Nurbs, test_bezier_quasi_interpolation_clipping)
{
    varmesh<2> m(3, 3);
    m[0][0] = v2{{-1, -1}};
    m[0][1] = v2{{0, -1}};
    m[0][2] = v2{{1, -1}};
    m[1][0] = v2{{-1, 0}};
    m[1][1] = v2{{0, 0}};
    m[1][2] = v2{{1, 0}};
    m[2][0] = v2{{-1, 1}};
    m[2][1] = v2{{0, 1}};
    m[2][2] = v2{{1, 1}};
    
    auto result = bezier_quasi_interpolation_clipping(m, 0.000001);
    ASSERT_EQ(1, result.size());
    auto expected = v2{{0.5, 0.5}};
    EXPECT_EQ(expected, result[0]);
}

TEST(Nurbs, test_bezier_max_deviation_to_quasi_interpolation_mesh)
{
    varmesh<4> m(3, 3);
    
    m[0][0] = v4{{-1.5, 1.5, 1, 1}};
    m[0][1] = v4{{0, 0, 1, 5}};
    //m[0][1] = v4{{0, 0, 1.0/5, 1}};
    m[0][2] = v4{{1.5, -0.5, 1, 1}};
    
    m[1][0] = v4{{-1, 0.5, 2, 1}};
    m[1][1] = v4{{0, -0.5, 2, 1}};
    m[1][2] = v4{{1, 0.5, 2, 1}};
    
    m[2][0] = v4{{-1, -3, 3, 1}};
    m[2][1] = v4{{0, -2, 15, 5}};
    //m[2][1] = v4{{0, -2.0/5, 3, 1}};
    m[2][2] = v4{{1, -1, 3, 1}};
    
    auto quasi_interpolating_polygon = bezier_surface_quasi_interpolation(m);
    
    double max_deviation = bezier_max_deviation_to_quasi_interpolation(m);
    
    double max_diff = 0;
    
    for (double u = 0; u <= 1; u += 0.1)
    {
        for (double v = 0; v <= 1; v += 0.1)
        {
            auto pp = evaluate_polygon_surface(quasi_interpolating_polygon, u, v);
            auto ppt = evaluate_polygon_surface(transpose(quasi_interpolating_polygon), v, u);
            
            expect_near(pp, ppt);
            
            double diff = l_inf(evaluate_polygon_surface(quasi_interpolating_polygon, u, v) - evaluate_bezier_surface(m, u, v));
        
            if (diff > max_diff)
            {
                max_diff = diff;
            }
            
            
        }
    }
    
    EXPECT_LE(max_diff, max_deviation);
}

TEST(Nurbs, test_bezier_clip_umin)
{
    std::vector<v<1>> p;
    p.push_back(v1{{1}});
    p.push_back(v1{{2}});
    p.push_back(v1{{4}});
    
    
    for (double u = 0; u <= 1; u += 0.1)
    {
    auto resultf = p;
    
        bezier_clip_curve<std::vector<v1>>(resultf, u, 1);
    
    auto result = p;
    
    
    result = subdivide_bezier_curve(result, u).second;
    
    EXPECT_EQ(result, resultf);
    }
}

TEST(Nurbs, test_bezier_clip_umin_umax)
{
    std::vector<v<1>> p;
    p.push_back(v1{{10}});
    p.push_back(v1{{2}});
    p.push_back(v1{{400}});
    
    
    for (double u = 0; u <= 1; u += 0.1)
    {
        for (double uu = u; uu <= 1; uu+=0.1)
        {
        auto resultf = p;
        
        bezier_clip_curve(resultf, u, uu);
        
        auto result = p;
        result = subdivide_bezier_curve(result, uu).first;
            result = subdivide_bezier_curve(result, (uu == 0 ? 0 :u/uu)).second;
        EXPECT_EQ(result, resultf);
        }
    }
}

TEST(Nurbs, test_bezier_clip_umin_umax_bug)
{
    std::vector<v<1>> p;
    p.push_back(v1{{1}});
    p.push_back(v1{{2}});
    p.push_back(v1{{4}});
    
    
            auto resultf = p;
            
            bezier_clip_curve(resultf, 0.25, 0.75);
            
            auto result = p;
            result = subdivide_bezier_curve(result, 0.75).first;
            result = subdivide_bezier_curve(result, 0.25/0.75).second;
            EXPECT_EQ(result, resultf);
}

TEST(Nurbs, test_raytrace_scene_multithreaded_empty)
{
    scene_descriptor scene = {
    512,
    512,
    v3{0, 0, -5},
    0, 0, 0,
    30,
    v3{1, 5, 1}
    };

    int count_calls = 0;
    auto trace_ray_do_nothing = [&count_calls](std::vector<int>::iterator, v3, scene_descriptor& scene) { count_calls++; };

    raytrace_scene_multithreaded<scene_descriptor>(scene, trace_ray_do_nothing, 1);

    EXPECT_EQ(512 * 512, count_calls);
}

