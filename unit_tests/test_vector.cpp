#include <gtest/gtest.h>

#include <geometry/types/bezier.h>

using ::testing::InitGoogleTest;
using ::testing::Test;
using ::testing::TestCase;
using ::testing::TestEventListeners;
using ::testing::TestInfo;
using ::testing::TestPartResult;
using ::testing::UnitTest;


TEST(Nurbs, vectorAddition)
{
	v2 p{ {3, 4} };
	v2 q{ {5, 6} };

	v2 result = p + q;
	v2 expected{ {8, 10} };
	EXPECT_EQ(expected, result);
}

TEST(Nurbs, vectorSubtract)
{
	v2 p{ {3, 6} };
	v2 q{ {5, 1} };

	v2 result = p - q;
	v2 expected{ {-2, 5} };
	EXPECT_EQ(expected, result);
}

TEST(Nurbs, vectorDotProduct)
{
	v2 p{ {3, 4} };
	v2 q{ {5, 6} };

	double result = p * q;
	EXPECT_EQ(39, result);
}

TEST(Nurbs, vectorScaling)
{
	v2 v{ {1, 2} };

	v2 expected{ {4, 8} };

	EXPECT_EQ(expected, 4 * v);
}

TEST(Nurbs, vectorNormalize)
{
	v2 v{ {3, 0} };

	EXPECT_EQ(3, length(v));

	v2 expected{ {1, 0} };

	EXPECT_EQ(expected, normalize(v));
}

TEST(Nurbs, vectorAddDimension)
{
	v2 v{ {3,4} };
	v3 result = add_dimension(v);

	v3 expected{ {3,4,1} };

	EXPECT_EQ(expected, result);
}

TEST(Nurbs, vectorRemoveDimension)
{
	v3 v{ {10, -4, 2} };
	v2 result = remove_dimension(v);

	v2 expected{ {5,-2} };

	EXPECT_EQ(expected, result);
}

TEST(Nurbs, hesse)
{
	v3 expected_hesse{ {1., 0., -1.} };
	v3 hesse = hesse_from(v2{ {1., 0.} }, v2{ {1., 1.} });
	EXPECT_EQ(expected_hesse, hesse);
}

TEST(Nurbs, test_convex_combination)
{
	v2 k{ {1, 2} };
	v2 l{ {4, 7} };

	v2 expected{ {2.5, 4.5} };
	EXPECT_EQ(expected, convex_combination(k, l, 0.5));

	expected = v2{ {1, 2} };
	EXPECT_EQ(expected, convex_combination(k, l, 0.));

	expected = v2{ {4, 7} };
	EXPECT_EQ(expected, convex_combination(k, l, 1.));
}

TEST(Nurbs, test_convex_combine_points)
{
	v2 k{ {1, 2} };
	v2 l{ {4, 7} };
	v2 m{ {2, 3} };

	std::vector<v2> given{ k, l, m };

	v2 expected1{ {2.5, 4.5} };
	v2 expected2{ {3, 5} };
	std::vector<v2> expected{ expected1, expected2 };

	EXPECT_EQ(expected, convex_combine_points(given, 0.5));
}

TEST(Nurbs, test_distance_from_plane)
{
	auto plane = vertical_plane_of_ray(v3{ {-2, 0, -5} }, v3{ {0, 0, 1} });

	auto point = v4{ {-4, 20, 1000, 1} };

	auto distance = point * plane;

	EXPECT_EQ(-2, distance);
}

TEST(Mesh, test_get_elements)
{
	varmesh<2> m(2, 2);

	m[0][0] = { 1, 2 };
	m[0][1] = { 2, 3 };
	m[1][0] = { 3, 4 };
	m[1][1] = { 4, 5 };

	EXPECT_EQ((v2{ 1, 2 }), m[0][0]);
	EXPECT_EQ((v2{ 2, 3 }), m[0][1]);
	EXPECT_EQ((v2{ 3, 4 }), m[1][0]);
	EXPECT_EQ((v2{ 4, 5 }), m[1][1]);

	for (int i = 0; i < m.row_size(); i++)
	{
		for (int j = 0; j < m.col_size(); j++)
		{
			EXPECT_EQ(m[i][j], m.element(i, j));
		}
	}
}

