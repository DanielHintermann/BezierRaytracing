#ifndef file_io_hpp
#define file_io_hpp

#include <tuple>
#include <filesystem>

#include <geometry/types/vector.h>

void serialize_as_ppm(const std::filesystem::path &file_path, int image_width, int image_height, const std::vector<int> &pixel);

std::pair<std::vector<v3>, std::vector<std::array<int, 3>>> parse_wavefront(std::string file_path);

std::string read_text_file(const std::filesystem::path& file);

#endif /* file_io_hpp */
