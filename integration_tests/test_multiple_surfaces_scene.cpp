#include <gtest/gtest.h>

#include "scene_setup.h"


using ::testing::InitGoogleTest;
using ::testing::Test;
using ::testing::TestCase;
using ::testing::TestEventListeners;
using ::testing::TestInfo;
using ::testing::TestPartResult;
using ::testing::UnitTest;

TEST(MultipleSurfacesScene, test_scene)
{
    auto scene = get_multiple_surfaces_scene();

    auto [pixel, width, height] = raytrace_scene_through_quasi_interpolation_multithreaded(scene, 12);

    auto file_name = GET_TEST_NAME + ".ppm";
    serialize_as_ppm(get_actual_folder() / file_name, width, height, pixel);
}