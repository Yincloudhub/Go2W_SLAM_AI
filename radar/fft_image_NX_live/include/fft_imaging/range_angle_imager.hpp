#pragma once

#include "fft_imaging/types.hpp"

namespace fft_imaging {

struct SceneGridConfig {
    double rmax_m = 40.0;
    double prf_hz = 9660.0;
    double c_mps = 3.0e8;
    double fc_hz = 76.0e9;
    double fs_hz = 22.5e6;
    double beam_width_deg = 120.0;
    double squint_angle_deg = 0.0;
    double grid_spacing_m = 0.03;
};

struct RangeAngleGeometry {
    VectorXd rr_m;
    VectorXd fa_hz;
    VectorXd aq_deg;
    VectorXd ax_m;
    VectorXd ay_m;
    Eigen::MatrixXd aq_grid_deg;
    Eigen::MatrixXd ar_grid_m;
    Eigen::ArrayXXd mask;
};

struct RangeAngleRemapPlan {
    Eigen::Index nr = 0;
    Eigen::Index na = 0;
    double v_mps = 0.0;
    SceneGridConfig config;
    RangeAngleGeometry geometry;
};

struct RemapBuffers {
    MatrixXc remapped_image;
    MatrixXc remapped_fliplr_image;
};

struct RemapResult {
    MatrixXc remapped_image;
    MatrixXc remapped_fliplr_image;
    RangeAngleGeometry geometry;
};

enum class InterpolationMode {
    Bilinear,
    Bicubic,
    Spline,
};

double derive_lambda_m(double c_mps, double fc_hz);
double derive_chirp_rate_hz_per_s(double fs_hz, double c_mps, double rmax_m);

RangeAngleGeometry build_range_angle_geometry(
    Eigen::Index nr,
    Eigen::Index na,
    const SceneGridConfig& config,
    double v_mps);

RangeAngleRemapPlan build_range_angle_remap_plan(
    Eigen::Index nr,
    Eigen::Index na,
    const SceneGridConfig& config,
    double v_mps);

void resize_remap_buffers(const RangeAngleRemapPlan& plan, RemapBuffers& buffers);

void remap_fft_to_range_angle(
    const MatrixXc& fft_image,
    const RangeAngleRemapPlan& plan,
    InterpolationMode interpolation_mode,
    RemapBuffers& buffers);

RemapResult remap_fft_to_range_angle(
    const MatrixXc& fft_image,
    const SceneGridConfig& config,
    double v_mps,
    InterpolationMode interpolation_mode = InterpolationMode::Bicubic);

}  // namespace fft_imaging
