#include "fft_imaging/vfit_reader.hpp"

#include <matio.h>

#include <cstring>
#include <stdexcept>
#include <string>

namespace fft_imaging {

namespace {

void validate_numeric_vector(const matvar_t* var, const std::string& variable_name) {
    if (var == nullptr) {
        throw std::runtime_error("MAT variable not found: " + variable_name);
    }

    if (var->class_type != MAT_C_DOUBLE) {
        throw std::runtime_error("MAT variable is not double precision: " + variable_name);
    }

    if (var->isComplex) {
        throw std::runtime_error("MAT variable must be real-valued for Vfit: " + variable_name);
    }

    if (var->rank < 2 || var->dims == nullptr) {
        throw std::runtime_error("MAT variable has invalid dimensions: " + variable_name);
    }
}

}  // namespace

VectorXd read_vfit_mat(
    const std::filesystem::path& mat_path,
    const std::string& variable_name) {
    mat_t* mat = Mat_Open(mat_path.string().c_str(), MAT_ACC_RDONLY);
    if (mat == nullptr) {
        throw std::runtime_error("Failed to open MAT file: " + mat_path.string());
    }

    matvar_t* var = Mat_VarRead(mat, variable_name.c_str());
    try {
        validate_numeric_vector(var, variable_name);

        std::size_t element_count = 1;
        for (int i = 0; i < var->rank; ++i) {
            element_count *= var->dims[i];
        }
        if (element_count == 0) {
            throw std::runtime_error("Vfit variable is empty: " + variable_name);
        }

        const auto* data = static_cast<const double*>(var->data);
        if (data == nullptr) {
            throw std::runtime_error("MAT variable has null data pointer: " + variable_name);
        }

        VectorXd output(static_cast<Eigen::Index>(element_count));
        for (Eigen::Index i = 0; i < output.size(); ++i) {
            output(i) = data[i];
        }

        Mat_VarFree(var);
        Mat_Close(mat);
        return output;
    } catch (...) {
        if (var != nullptr) {
            Mat_VarFree(var);
        }
        Mat_Close(mat);
        throw;
    }
}

}  // namespace fft_imaging
