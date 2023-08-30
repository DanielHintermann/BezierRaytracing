#include <gtest/gtest.h>

#include <geometry/types/bezier.h>

#include <geometry/algorithms/quasi_interpolation.h>
#include <geometry/types/bezier_surface.h>

using ::testing::InitGoogleTest;
using ::testing::Test;
using ::testing::TestCase;
using ::testing::TestEventListeners;
using ::testing::TestInfo;
using ::testing::TestPartResult;
using ::testing::UnitTest;

varmesh<2> get_bilinear_patch()
{
	/*
		[0]	-5.0299731099712508	double
		[1]	2.5515510043108915	double
		
		[0]	-2.0361692235851914	double
		[1]	2.5515510043108915	double
		
		[0]	-1.7859714022930659	double
		[1]	-1.1647750360679388	double
		
		[0]	1.2078324840929935	double
		[1]	-1.1647750360679388	double


*/
	varmesh<2> m(2, 2);
	m.element(0, 0) = { -5.0299731099712508, 2.5515510043108915 };
	m.element(0, 1) = { -2.0361692235851914, 2.5515510043108915 };
	m.element(1, 0) = { -1.7859714022930659, -1.1647750360679388 };
	m.element(1, 1) = { 1.2078324840929935, -1.1647750360679388 };

	return m;
}

varmesh<2> get_other_bilinear_patch()
{
	/*
	-		[0]	{ size=2 }	std::array<double,2>
		[0]	-2.9094401716819789	double
		[1]	8.4614733038636381	double
+		[Raw View]	{_Elems=0x0000023f6ff76c30 {-2.9094401716819789, 8.4614733038636381} }	std::array<double,2>
-		[1]	{ size=2 }	std::array<double,2>
		[0]	0.089573925121074732	double
		[1]	8.4614733038636381	double
+		[Raw View]	{_Elems=0x0000023f6ff76c40 {0.089573925121074732, 8.4614733038636381} }	std::array<double,2>
-		[2]	{ size=2 }	std::array<double,2>
		[0]	-1.6148652130335637	double
		[1]	-0.66230593116212177	double
+		[Raw View]	{_Elems=0x0000023f6ff76c50 {-1.6148652130335637, -0.66230593116212177} }	std::array<double,2>
-		[3]	{ size=2 }	std::array<double,2>
		[0]	1.3841488837694897	double
		[1]	-0.66230593116212177	double

	*/

	varmesh<2> m(2, 2);
	m.element(0, 0) = { -2.9094401716819789, 8.4614733038636381 };
	m.element(0, 1) = { 0.089573925121074732, 8.4614733038636381 };
	m.element(1, 0) = { -1.6148652130335637, -0.66230593116212177 };
	m.element(1, 1) = { 1.3841488837694897, -0.66230593116212177 };

	return m;
}

varmesh<2> get_simple_bilinear_patch()
{
	varmesh<2> m(2, 2);
	m.element(0, 0) = { -1, -1 };
	m.element(0, 1) = { -1, 1 };
	m.element(1, 0) = { 1, -1 };
	m.element(1, 1) = { 1, 1 };

	return m;
}

TEST(BilinearPatchIntersection, test_other_patch_roots_clipping)
{
	auto result = bilinear_patch_roots_clipping(get_other_bilinear_patch(), 1E-8);

	EXPECT_EQ(result.size(), 1);
	if (result.size() == 1)
	{
		auto y = evaluate_bezier_surface(get_other_bilinear_patch(), result[0][0], result[0][1]);

		EXPECT_EQ((v2{ 0, 0 }), y);
	}
}

TEST(BilinearPatchIntersection, test_clip_patch)
{
	auto m = get_simple_bilinear_patch();

	auto cm{m};

	bezier_clip_surface(cm, v2{ 0, 0.5 }, v2{ 0, 0.5 });

	std::cout << "";
}

TEST(BilinearPatchIntersection, test_eval_patch)
{
	auto m = get_simple_bilinear_patch();

	auto y = evaluate_bezier_surface(m, 0, 0.5);
	y = evaluate_bezier_surface(m, 1, 0.5);
	y = evaluate_bezier_surface(m, 0.5, 0);
	y = evaluate_bezier_surface(m, 0.5, 1);
	std::cout << "";
}

TEST(BilinearPatchIntersection, test_simple_patch_roots)
{
	auto result = bilinear_patch_roots(get_simple_bilinear_patch(), 1E-8);

	EXPECT_EQ(result.size(), 1);

	auto y = evaluate_bezier_surface(get_simple_bilinear_patch(), result[0][0], result[0][1]);

	EXPECT_EQ((v2{ 0, 0 }), y);
}

TEST(BilinearPatchIntersection, test_patch_roots)
{
	auto result = bilinear_patch_roots(get_bilinear_patch(), 1E-8);

	EXPECT_EQ(result.size(), 1);

	auto y = evaluate_bezier_surface(get_bilinear_patch(), result[0][0], result[0][1]);

	EXPECT_EQ((v2{ 0, 0 }), y);
}

TEST(BilinearPatchIntersection, test_simple_patch_roots_clipping)
{
	auto result = bilinear_patch_roots_clipping(get_simple_bilinear_patch(), 1E-8);

	EXPECT_EQ(result.size(), 1);

	auto y = evaluate_bezier_surface(get_simple_bilinear_patch(), result[0][0], result[0][1]);

	EXPECT_EQ((v2{ 0, 0 }), y);
}

TEST(BilinearPatchIntersection, test_patch_roots_clipping)
{
	auto result = bilinear_patch_roots_clipping(get_bilinear_patch(), 1E-8);

	EXPECT_EQ(result.size(), 1);
	if (result.size() == 1)
	{
		auto y = evaluate_bezier_surface(get_bilinear_patch(), result[0][0], result[0][1]);

		EXPECT_EQ((v2{ 0, 0 }), y);
	}
}

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

TEST(Bezier, test_surface_derivatives)
{
	varmesh<4> m(3, 4);

	m.element(0, 0) = { 0, 0, 6, 1 };
	m.element(0, 1) = { 3, 0, 0, 1 };
	m.element(0, 2) = { 6, 0, 0, 1 };
	m.element(0, 3) = { 9, 0, 6, 1 };

	m.element(1, 0) = { 0, 3, 3, 1 };
	m.element(1, 1) = { 3, 3, 0, 1 };
	m.element(1, 2) = { 6, 3, 0, 1 };
	m.element(1, 3) = { 9, 3, 0, 1 };

	m.element(2, 0) = { 0, 6, 6, 1 };
	m.element(2, 1) = { 3, 6, 0, 1 };
	m.element(2, 2) = { 6, 6, 0, 1 };
	m.element(2, 3) = { 9, 6, 6, 1 };

	double u = 0.5;
	double v = 0.5;

	auto curve_u = bezier_curve_on_surface_u(m, v);

	auto derivative_u = evaluate_rational_bezier_curve_derivative(curve_u, u);

	auto curve_v = bezier_curve_on_surface_v(m, u);

	auto derivative_v = evaluate_rational_bezier_curve_derivative(curve_v, v);

	EXPECT_EQ(derivative_u, (v4{ 9, 0, -1.125, 1 }));
	EXPECT_EQ(derivative_v, (v4{ 0, 6, 0, 1 }));

	auto n = cross_product(remove_dimension(derivative_u), remove_dimension(derivative_v));

	auto nn = normalize(n);

	//EXPECT_EQ(nn, (v3{-0.1240, 0, -0.9922}));

	double eps = 1E-8;

	auto y = evaluate_bezier_surface(m, u, v);
	auto yu = evaluate_bezier_surface(m, u + eps, v);
	auto yv = evaluate_bezier_surface(m, u, v + eps);

	double inv_eps = 1.0 / eps;

	auto adu = inv_eps*(yu - y);
	adu[3] = 1;

	auto adv = inv_eps * (yv - y);
	adv[3] = 1;

	auto an = cross_product(remove_dimension(adu), remove_dimension(adv));

	auto ann = normalize(an);

	EXPECT_EQ(ann, (v3{-0.1240, 0, -0.9922}));
}

