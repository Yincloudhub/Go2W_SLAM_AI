#include "fft_imaging/matrix_summary.hpp"

#include <sstream>

namespace fft_imaging {

std::string describe_matrix(const MatrixXc& matrix, const std::string& name) {
    std::ostringstream out;
    out << name
        << ": rows=" << matrix.rows()
        << ", cols=" << matrix.cols()
        << ", type=std::complex<double>";
    return out.str();
}

}  // namespace fft_imaging
