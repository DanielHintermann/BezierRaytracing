#include <fstream>
#include <sstream>

#include "file_io.h"

void serialize_as_ppm(const std::filesystem::path &file_path, int image_width, int image_height, const std::vector<int> &pixel)
{
    std::ofstream open_file(file_path, std::ofstream::trunc);
    
    open_file<<"P3"<<std::endl<<image_width<<" "<<image_height<<std::endl<<"255"<<std::endl;

    for (auto current_pixel = pixel.begin(); current_pixel != pixel.end(); current_pixel++)
    {
        open_file << *current_pixel << " ";
    }
}

std::pair<std::vector<v3>, std::vector<std::array<int, 3>>> parse_wavefront(std::string file_path)
{
    std::ifstream infile(file_path);
    
    std::vector<v3> points;
    std::vector<std::array<int, 3>> facets;
    char type;
    while (infile>>type)
    {
        if ('v' == type)
        {
            v3 v;
            if (infile>>v[0]>>v[1]>>v[2])
            {
                points.push_back(v);
            }
        }
        if ('f' == type)
        {
            std::array<int, 3> f;
            if (infile>>f[0]>>f[1]>>f[2])
            {
                f[0]--; f[1]--; f[2]--;
                facets.push_back(f);
            }
        }
    }
    
    return std::make_pair(points, facets);
}

std::string read_text_file(const std::filesystem::path& file)
{
    std::stringstream str;
    std::ifstream stream(file);

    if (stream.is_open())
    {
        while (stream.peek() != EOF)
        {
            str << (char)stream.get();
        }
        stream.close();

        return str.str();
    }

    return "";
}

