#pragma once

#include <filesystem>
#include <string>

#include "fft_imaging/types.hpp"

namespace fft_imaging {

VectorXd read_vfit_mat(
    const std::filesystem::path& mat_path,
    const std::string& variable_name = "Vfit");

}  // namespace fft_imaging
