#include "scene_setup.h"

#include <gtest/gtest.h>

int threads_to_use()
{
    return std::thread::hardware_concurrency();
}

void compare_actual_with_expected_file(const std::string file_name)
{
    auto actual = read_text_file(get_actual_folder() / file_name);
    auto expected = read_text_file(get_expected_folder() / file_name);

    EXPECT_EQ(expected, actual);
}

std::filesystem::path get_actual_folder()
{
    return std::filesystem::path(__FILE__).parent_path() / "actual";
}

std::filesystem::path get_expected_folder()
{
    return std::filesystem::path(__FILE__).parent_path() / "expected";
}

std::function<v3(double, double)> get_texture(int index)
{
    switch (index)
    {
    case 0:
        return [](double u, double v) {
            v2 uv_parameter{ {u, v} };
            int color_index = ((int)std::round(uv_parameter[0] * 10) + (int)std::round(uv_parameter[1] * 10)) % 2;
            if (0 > color_index)
            {
                color_index = 0;
            }

            if (0 == color_index)
                return v3{ {1, 210./256, 80./256}};
            else
                return v3{ {210. / 256, 80. / 256, 255. / 256}};
        };
    case 1:
        return [](double u, double v) {
            v2 uv_parameter{ {u, v} };
            int color_index = ((int)std::round(uv_parameter[0] * 10) + (int)std::round(uv_parameter[1] * 10)) % 2;
            if (0 > color_index)
            {
                color_index = 0;
            }

            if (0 == color_index)
                return v3{ {255./255, 80. / 255, 210. / 255}};
            else
                return v3{ {25. / 255, 255. / 255, 45. / 255}};
        };
    default:
        return [](double u, double v) { return v3{0, 0, 0}; };
    }
}

varmesh<4> get_curved_patch()
{
    varmesh<4> m(3, 3);

    m[0][0] = v4{ {-1.5, 1.5, 1, 1} };
    m[0][1] = v4{ {0, 0, 1, 5} };
    //m[0][1] = v4{{0, 0, 1.0/5, 1}};
    m[0][2] = v4{ {1.5, -0.5, 1, 1} };

    m[1][0] = v4{ {-1, 0.5, 2, 1} };
    m[1][1] = v4{ {0, -0.5, 2, 1} };
    m[1][2] = v4{ {1, 0.5, 2, 1} };

    m[2][0] = v4{ {-1, -3, 3, 1} };
    m[2][1] = v4{ {0, -2, 15, 5} };
    //m[2][1] = v4{{0, -2.0/5, 3, 1}};
    m[2][2] = v4{ {1, -1, 3, 1} };

    return m;
}

varmesh<4> get_sphere_patch()
{
    varmesh<4> m(5, 5);

    auto s = sqrt(2);

    m[0][0] = 8 * v4{ 8, 0, 0, 1 };
    m[0][1] = (4 * s) * v4{ 4 * s, 2 * s, 0, 1 };
    m[0][2] = (16. / 3) * v4{ 4, 4, 0, 1 };
    m[0][3] = (4 * s) * v4{ 2 * s, 4 * s, 0, 1 };
    m[0][4] = 8 * v4{ 0, 8, 0, 1 };

    m[1][0] = (4 * s) * v4{ 4 * s, -2 * s, 0, 1 };
    m[1][1] = 3 * v4{ 4, 0, -3, 1 };
    m[1][2] = (4 * s / 3) * v4{ 5 * s / 3, 5 * s / 3, -8 * s / 3, 1 };
    m[1][3] = 3 * v4{ 0, 4, -3, 1 };
    m[1][4] = (4 * s) * v4{ -2 * s, 4 * s, 0, 1 };

    m[2][0] = (16 / 3) * v4{ 4, -4, 0, 1 };
    m[2][1] = (4 * s / 3) * v4{ 5 * s / 3, -5 * s / 3, -8 * s / 3, 1 };
    m[2][2] = (8. / 9) * v4{ 0, 0, -16. / 3, 1 };
    m[2][3] = (4 * s / 3) * v4{ -5 * s / 3, 5 * s / 3, -8 * s / 3, 1 };
    m[2][4] = (16. / 3) * v4{ -4, 4, 0, 1 };

    m[3][0] = (4 * s) * v4{ 2 * s, -4 * s, 0, 1 };
    m[3][1] = 3 * v4{ 0, -4, -3, 1 };
    m[3][2] = (4 * s / 3) * v4{ -5 * s / 3, -5 * s / 3, -8 * s / 3, 1 };
    m[3][3] = 3 * v4{ -4, 0, -3, 1 };
    m[3][4] = (4 * s) * v4{ -4 * s, 2 * s, 0, 1 };

    m[4][0] = 8 * v4{ 0, -8, 0, 1 };
    m[4][1] = (4 * s) * v4{ -2 * s, -4 * s, 0, 1 };
    m[4][2] = (16. / 3) * v4{ -4, -4, 0, 1 };
    m[4][3] = (4 * s) * v4{ -4 * s, -2 * s, 0, 1 };
    m[4][4] = 8 * v4{ -8, 0, 0, 1 };

    auto r = rotation_in_e2(3.14 * 25 / 180) * rotation_in_e1(3.14 * 60 / 180);

    for (int i = 0; i < m.row_size(); i++)
        for (int j = 0; j < m.col_size(); j++)
            m[i][j] = r * m[i][j];

    return m;
}

varmesh<4> get_twisted_patch()
{
    varmesh<4> m(2, 2);

    m[0][0] = v4{ {-2, 0, 1, 1} };
    m[0][1] = v4{ {2, 0, 1, 1} };

    m[1][0] = v4{ {0, -2, 2, 1} };
    m[1][1] = v4{ {0, 2, 2, 1} };

    return m;
}

varmesh<4> get_twisted_patch3()
{
    varmesh<4> m(3, 3);

    m[0][0] = v4{ {-2, 0, 1, 1} };
    m[0][1] = v4{ {0, 0, 1, 1} };
    m[0][2] = v4{ {2, 0, 1, 1} };

    m[1][0] = v4{ {-1, -1, 1.5, 1} };
    m[1][1] = v4{ {0, 0, 1, 1.5} };
    m[1][2] = v4{ {1, 1, 1, 1.5} };

    m[2][0] = v4{ {0, -2, 2, 1} };
    m[2][1] = v4{ {0, 0, 2, 1} };
    m[2][2] = v4{ {0, 2, 2, 1} };

    return m;
}

multiple_surfaces_scene_descriptor get_curved_patch_scene()
{
    multiple_surfaces_scene_descriptor scene = {
    screen_geometry(1024, 1024, 30, 0, 0, 0),
    v3{0, 0, -5},
    v3{-1, 2, -5}, 10,
    std::vector< std::shared_ptr<scene_object>>{
        std::make_shared<varmesh_scene_object>(get_curved_patch(), get_texture(0), diffuse)},
    1E-5
    };

    return scene;
}

multiple_surfaces_scene_descriptor get_twisted_patch_scene()
{
    multiple_surfaces_scene_descriptor scene = {
    screen_geometry(1024, 1024, 30, 0, 0, 0),
    v3{0, 0, -5},
    v3{-1, 2, -5}, 10,
    std::vector< std::shared_ptr<scene_object>>{
        std::make_shared<varmesh_scene_object>(get_twisted_patch(), get_texture(0), diffuse)},
    1E-9
    };

    return scene;
}

multiple_surfaces_scene_descriptor get_sphere_scene()
{
    multiple_surfaces_scene_descriptor scene = {
    screen_geometry(1024, 1024, 30, 0, 0, 0),
    v3{0, 0, -35},
    -1 * v3{-1, 1, -5}, 10,
    std::vector< std::shared_ptr<scene_object>>{
        std::make_shared<varmesh_scene_object>(get_sphere_patch(), get_texture(0), diffuse)},
    1E-5
    };

    return scene;
}

multiple_surfaces_scene_descriptor get_multiple_surfaces_scene()
{
    varmesh<4> plane(3, 3);

    plane[0][0] = v4{ {-20.5, 6, 50, 1} };
    plane[0][1] = v4{ {0, 3, 50, 1} };
    plane[0][2] = v4{ {20.5 ,6, 50, 1} };

    plane[1][0] = v4{ {-20.5, 2.0, 25, 1} };
    plane[1][1] = 1.4*v4{ {0, -6.5, 25, 1} };
    plane[1][2] = v4{ {20.5 ,1.6, 25, 1} };

    plane[2][0] = v4{ {-20.5, 2, -0.5, 1} };
    plane[2][1] = 0.9*v4{ {0, -4.5, -0.5, 1} };
    plane[2][2] = v4{ {20.5, 2, -0.5, 1} };

    auto objects = std::vector<std::shared_ptr<scene_object>>{
        std::make_shared<sphere_scene_object>(v3{-1, 1, 3}, 1, get_texture(0), diffuse),
        std::make_shared<sphere_scene_object>(v3{1.5, 0, 4}, 0.7, get_texture(1), reflective),
        std::make_shared<sphere_scene_object>(v3{0, -0.5, 5}, 0.9, get_texture(1), diffuse),
        //std::make_shared<varmesh_scene_object>(move(scale(get_curved_patch(), -0.6), v3{0.4, 0.4, 3}), get_texture(0), diffuse),
        //std::make_shared<varmesh_scene_object>(move(scale(get_curved_patch(), 0.6), v3{-0.3, -0.3, 2}), get_texture(1), diffuse),
        std::make_shared<varmesh_scene_object>(plane, get_texture(0), reflective)
    };

    return multiple_surfaces_scene_descriptor {
        screen_geometry(1024, 1024, 30, 0, 0, 0),
            v3{ 0, 0, -5 },
            v3{ 5, 10, -5 }, 4,
                objects,
                1E-8
    };
}

multiple_surfaces_scene_descriptor get_multiple_splitted_surfaces_scene()
{
    varmesh<4> plane(2, 2);

    plane[0][0] = v4{ {-1.5, -1.5, 1, 1} };
    plane[0][1] = v4{ {1.5,-1.5, 1, 1} };

    plane[1][0] = v4{ {-1.5, -1.5, -0.5, 1} };
    plane[1][1] = v4{ {1.5, -1.5, -0.5, 1} };

    auto objects = std::vector< std::shared_ptr<scene_object>>{
        std::make_shared<varmesh_scene_object>(move(scale(get_curved_patch(), -0.6), v3{0.4, 0.4, 3}), get_texture(0), diffuse),
        std::make_shared<varmesh_scene_object>(move(scale(get_curved_patch(), 0.6), v3{-0.3, -0.3, 2}), get_texture(1), reflective),
        std::make_shared<varmesh_scene_object>(plane, get_texture(0), reflective)
    };

    for (int i = 1; i < 50; i++)
    {
        varmesh<4> p(2, 2);

        p[0][0] = v4{ {-1.5, -1.5, i+1., 1} };
        p[0][1] = v4{ {1.5,-1.5, i+1., 1} };

        p[1][0] = v4{ {-1.5, -1.5, i + 0., 1} };
        p[1][1] = v4{ {1.5, -1.5, i + 0., 1} };

        objects.push_back(std::make_shared<varmesh_scene_object>(p, get_texture(0), diffuse));
    }

    return multiple_surfaces_scene_descriptor{
        screen_geometry(1024, 1024, 30, 0, 0, 0),
            v3{ 0, 0, -5 },
            v3{ 5, 10, -5 }, 10, 
                objects,
                1E-8
                
    };
}