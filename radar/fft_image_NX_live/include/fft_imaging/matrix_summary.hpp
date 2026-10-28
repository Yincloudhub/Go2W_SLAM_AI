#pragma once

#include <string>

#include "fft_imaging/types.hpp"

namespace fft_imaging {

std::string describe_matrix(const MatrixXc& matrix, const std::string& name);

}  // namespace fft_imaging
