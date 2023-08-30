#include <gtest/gtest.h>

#include "scene_setup.h"


using ::testing::InitGoogleTest;
using ::testing::Test;
using ::testing::TestCase;
using ::testing::TestEventListeners;
using ::testing::TestInfo;
using ::testing::TestPartResult;
using ::testing::UnitTest;

//TEST(MultipleSurfacesScene, test_plane_not_there)
//{
//    auto scene = get_multiple_surfaces_scene();
//
//    screen_geometry screen(scene.screen_width, scene.screen_height, scene.field_of_view, 0, 0, 0);
//
//    auto ray = screen.get_corresponding_ray(280, 431);
//
//    auto intersections = get_ray_surface_intersection(scene.origin, ray, scene, 1E-8);
//
//    EXPECT_EQ(true, intersections.has_value());
//}
//
//TEST(MultipleSurfacesScene, test_bug_in)
//{
//    auto scene = get_multiple_surfaces_scene();
//
//    screen_geometry screen(scene.screen_width, scene.screen_height, scene.field_of_view, 0, 0, 0);
//
//    auto ray = screen.get_corresponding_ray(317, 326);
//
//    auto intersections = get_ray_surface_intersection(scene.origin, ray, scene, 1E-8);
//
//    EXPECT_EQ(true, intersections.has_value());
//}
//
//TEST(MultipleSurfacesScene, test_bug_out)
//{
//    auto scene = get_multiple_surfaces_scene();
//
//    screen_geometry screen(scene.screen_width, scene.screen_height, scene.field_of_view, 0, 0, 0);
//
//    auto ray = screen.get_corresponding_ray(318, 326);
//
//    auto intersections = get_ray_surface_intersection(scene.origin, ray, scene, 1E-8);
//
//    EXPECT_EQ(true, intersections.has_value());
//}

TEST(MultipleSurfacesScene, test_bezier_scene)
{
    auto scene = get_multiple_surfaces_scene();

    auto [pixel, width, height] = raytrace_scene_through_quasi_interpolation_multithreaded(scene, threads_to_use());

    auto file_name = GET_TEST_NAME + ".ppm";
    serialize_as_ppm(get_actual_folder() / file_name, width, height, pixel);
}

//TEST(MultipleSurfacesScene, test_splitted_plane_scene)
//{
//    auto scene = get_multiple_splitted_surfaces_scene();
//
//    auto [pixel, width, height] = raytrace_scene_through_quasi_interpolation_multithreaded(scene, threads_to_use());
//
//    auto file_name = GET_TEST_NAME + ".ppm";
//    serialize_as_ppm(get_actual_folder() / file_name, width, height, pixel);
//}