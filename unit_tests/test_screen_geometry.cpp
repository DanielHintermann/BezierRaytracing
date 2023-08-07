#include <gtest/gtest.h>

#include <graphics/screen_geometry.h>

using ::testing::InitGoogleTest;
using ::testing::Test;
using ::testing::TestCase;
using ::testing::TestEventListeners;
using ::testing::TestInfo;
using ::testing::TestPartResult;
using ::testing::UnitTest;

TEST(ScreenGeometry, test_screen_geometry_consistency)
{
	int w = 1024;
	int h = 800;
	double fov = 30;

	screen_geometry s(w, h, fov, 0.1, 0.2, 0.3);

	for (int i = 0; i < w; i++)
	{
		for (int j = 0; j < h; j++)
		{
			auto ray = s.get_corresponding_ray(i, j);
			//ray = 10 * ray;

			auto [x, y] = s.get_corresponding_screen_pixel(ray);

			EXPECT_EQ(x, i);
			EXPECT_EQ(y, j);
		}
	}
}

TEST(ScreenGeometry, test_screen_geometry_aspectratio)
{
	int w = 2048;
	int h = 512;
	double fov = 30;

	screen_geometry s(w, h, fov, 0, 0, 0);

	auto ray_mid = s.get_corresponding_ray(1023, 255);

	auto ray_00 = s.get_corresponding_ray(1023, 255);
	auto ray_x0 = s.get_corresponding_ray(1024, 255);
	auto ray_0y = s.get_corresponding_ray(1023, 256);
	auto ray_xy = s.get_corresponding_ray(1024, 256);
	std::cout << "";
}