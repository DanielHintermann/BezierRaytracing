#include <gtest/gtest.h>

#include <fstream>
#include <filesystem>

#include <raytracing/nurbs_raytracing.h>
#include <geometry/types/bezier_surface.h>


using ::testing::InitGoogleTest;
using ::testing::Test;
using ::testing::TestCase;
using ::testing::TestEventListeners;
using ::testing::TestInfo;
using ::testing::TestPartResult;
using ::testing::UnitTest;

#define GET_TEST_NAME std::string(::testing::UnitTest::GetInstance()->current_test_info()->name())

varmesh<4> get_curved_patch()
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

    return m;
}

varmesh<4> get_sphere_patch()
{
    varmesh<4> m(5, 5);
    
    auto s = sqrt(2);

    m[0][0] = 8 * v4{8, 0, 0, 1};
    m[0][1] = (4*s) * v4{4*s, 2*s, 0, 1};
    m[0][2] = (16./3) * v4{4, 4, 0, 1};
    m[0][3] = (4*s) * v4{2*s, 4*s, 0, 1};
    m[0][4] = 8 * v4{0, 8, 0, 1};

    m[1][0] = (4*s) * v4{4*s, -2*s, 0, 1};
    m[1][1] = 3 * v4{4, 0, -3, 1};
    m[1][2] = (4*s/3)* v4{5*s/3, 5*s/3, -8*s/3, 1};
    m[1][3] = 3 * v4{0, 4, -3, 1};
    m[1][4] = (4*s)* v4{-2*s, 4*s, 0, 1};

    m[2][0] = (16/3) * v4{4, -4, 0, 1};
    m[2][1] = (4*s/3) * v4{5*s/3, -5*s/3, -8*s/3, 1};
    m[2][2] = (8./9)* v4{0, 0, -16./3, 1};
    m[2][3] = (4*s/3) * v4{-5*s/3, 5*s/3, -8*s/3, 1};
    m[2][4] = (16./3)* v4{-4, 4, 0, 1};

    m[3][0] = (4*s)* v4{2*s, -4*s, 0, 1};
    m[3][1] = 3* v4{0, -4, -3, 1};
    m[3][2] = (4*s/3)* v4{-5*s/3, -5*s/3, -8*s/3, 1};
    m[3][3] = 3 * v4{-4, 0, -3, 1};
    m[3][4] = (4*s)* v4{-4*s, 2*s, 0, 1};

    m[4][0] = 8 * v4{0, -8, 0, 1};
    m[4][1] = (4*s) * v4{-2*s, -4*s, 0, 1};
    m[4][2] = (16./3) * v4{-4, -4, 0, 1};
    m[4][3] = (4*s) * v4{-4*s, -2*s, 0, 1};
    m[4][4] = 8 * v4{-8, 0, 0, 1};

    return m;
}

varmesh_scene_descriptor get_curved_patch_scene()
{
    varmesh_scene_descriptor scene = {
    512,
    512,
    v3{0, 0, -5},
    30,
    v3{-1, 2, -5},
    get_curved_patch(),
    [](double u, double v) {
        v2 uv_parameter{{u, v}};
        int color_index = ((int)std::round(uv_parameter[0] * 10) + (int)std::round(uv_parameter[1] * 10)) % 2; 
        if (0 > color_index)
        {
            color_index = 0;
        }
        
        if (0 == color_index)
            return std::vector<int>{ {255, 210, 80}}; 
        else
            return std::vector<int>{ {210, 80, 255}};    
        }
    };

    return scene;
}

varmesh_scene_descriptor get_sphere_scene()
{
    varmesh_scene_descriptor scene = {
    512,
    512,
    v3{0, 0, -35},
    30,
    -1*v3{-1, 1, -5},
    get_sphere_patch(), 
    [](double u, double v) {
        v2 uv_parameter{{u, v}};
        int color_index = ((int)std::round(uv_parameter[0] * 10) + (int)std::round(uv_parameter[1] * 10)) % 2;
        if (0 > color_index)
        {
            color_index = 0;
        }

        if (0.25 >= length2(uv_parameter - v2{0.5, 0.5}))
        {
            if (0 == color_index)
                return std::vector<int>{ {255, 210, 80}};
            else
                return std::vector<int>{ {210, 80, 255}};

        }
        else
        {
            return std::vector<int>{ {0, 0, 0}};
        }
    }
    };

    return scene;
}

std::filesystem::path get_actual_folder()
{
    return std::filesystem::path(__FILE__).parent_path() / "actual";
}

std::filesystem::path get_expected_folder()
{
    return std::filesystem::path(__FILE__).parent_path() / "expected";
}

void compare_actual_with_expected_file(const std::string file_name)
{
    auto actual = read_text_file(get_actual_folder() / file_name);
    auto expected = read_text_file(get_expected_folder() / file_name);

    EXPECT_EQ(expected, actual);
}

TEST(Nurbs, test_sphere_patch)
{
    auto scene = get_sphere_scene();

    auto trace_ray_through_quasi_interpolation = [](std::vector<int>::iterator pixel, v3 ray, varmesh_scene_descriptor& scene) { 
        trace_ray(pixel, ray, scene);
    };

    auto [pixel, width, height] = raytrace_scene_multithreaded<varmesh_scene_descriptor>(scene, trace_ray_through_quasi_interpolation, 12);

    auto file_name = GET_TEST_NAME + ".ppm";
    serialize_as_ppm(get_actual_folder() / file_name, width, height, pixel);
    compare_actual_with_expected_file(file_name);
}

TEST(Nurbs, test_curved_patch_subdivision)
{
    auto scene = get_curved_patch_scene();

    auto [pixel, width, height] = raytrace_scene_with_meshes_hierarchy_multithreaded(scene, 12);

    auto file_name = GET_TEST_NAME + ".ppm";
    serialize_as_ppm(get_actual_folder() / file_name, width, height, pixel);
    compare_actual_with_expected_file(file_name);
}

TEST(Nurbs, test_teapot)
{
    auto teapot = std::filesystem::path(__FILE__).parent_path().parent_path() / "3d_models" / "teapot.obj";

    auto [facets, points] = parse_wavefront(teapot.string());

	EXPECT_EQ(3644, facets.size());
	EXPECT_EQ(6320, points.size());

    facetted_surface_scene_descriptor scene = { 640, 480, {14, 9, -12}, 30, {1, 1, 1}, points, facets };

    auto [pixel, width, height] = raytrace_scene_with_facetted_surface_multithreaded(scene, 12);

    auto file_name = GET_TEST_NAME + ".ppm";
    serialize_as_ppm(get_actual_folder() / file_name, width, height, pixel);
    compare_actual_with_expected_file(file_name);
}

TEST(Nurbs, test_curved_patch)
{   
    auto scene = get_curved_patch_scene();

    auto [pixel, width, height] = raytrace_scene_through_quasi_interpolation_multithreaded(scene, 12);

    auto file_name = GET_TEST_NAME + ".ppm";
    serialize_as_ppm(get_actual_folder() / file_name, width, height, pixel);
    compare_actual_with_expected_file(file_name);
}

TEST(Nurbs, test_planar_patch)
{   
    auto scene = get_curved_patch_scene();

    varmesh<4> plane(2, 2);

    plane[0][0] = v4{{-1.5, 1.5, 1, 1}};
    plane[0][1] = v4{{1.5, 0.5, 1, 1}};
    
    plane[1][0] = v4{{-1, -0.5, 2, 1}};
    plane[1][1] = v4{{1.5, -0.5, 2, 1}};

    scene.mesh = plane;

    auto [pixel, width, height] = raytrace_scene_through_quasi_interpolation_multithreaded(scene, 12);

    auto file_name = GET_TEST_NAME + ".ppm";
    serialize_as_ppm(get_actual_folder() / file_name, width, height, pixel);
    compare_actual_with_expected_file(file_name);
}
