#include <gtest/gtest.h>

#include "scene_setup.h"


using ::testing::InitGoogleTest;
using ::testing::Test;
using ::testing::TestCase;
using ::testing::TestEventListeners;
using ::testing::TestInfo;
using ::testing::TestPartResult;
using ::testing::UnitTest;

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

    facetted_surface_scene_descriptor scene = { 640, 480, {14, 9, -12}, 0, 0, 0, 30, {1, 1, 1}, points, facets };

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
