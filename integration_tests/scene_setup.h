#ifndef test_integration_scene_setup_h
#define test_integration_scene_setup_h

#include <fstream>
#include <filesystem>

#include <raytracing/nurbs_raytracing.h>
#include <geometry/types/bezier_surface.h>

#define GET_TEST_NAME std::string(::testing::UnitTest::GetInstance()->current_test_info()->name())

int threads_to_use();

void compare_actual_with_expected_file(const std::string file_name);

std::filesystem::path get_actual_folder();

std::filesystem::path get_expected_folder();

std::function<v3(double, double)> get_texture(int index);

varmesh<4> get_curved_patch();

varmesh<4> get_sphere_patch();

varmesh<4> get_twisted_patch();

multiple_surfaces_scene_descriptor get_curved_patch_scene();

multiple_surfaces_scene_descriptor get_sphere_scene();

multiple_surfaces_scene_descriptor get_twisted_patch_scene();

multiple_surfaces_scene_descriptor get_multiple_surfaces_scene();
multiple_surfaces_scene_descriptor get_multiple_splitted_surfaces_scene();

#endif // test_integration_scene_setup_h
