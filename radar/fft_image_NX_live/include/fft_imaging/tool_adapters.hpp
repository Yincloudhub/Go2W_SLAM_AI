#pragma once

#include <filesystem>
#include <iosfwd>

#include "fft_imaging/image_writer.hpp"
#include "fft_imaging/runtime.hpp"
#include "fft_imaging/vfit_reader.hpp"

namespace fft_imaging {

class VectorFrameVelocityProvider final : public FrameVelocityProvider {
public:
    explicit VectorFrameVelocityProvider(VectorXd velocities_mps);

    std::size_t available_frames() const override;
    double velocity_for_frame(std::size_t frame_index) const override;

private:
    VectorXd velocities_mps_;
};

class MatFileFrameVelocityProvider final : public FrameVelocityProvider {
public:
    explicit MatFileFrameVelocityProvider(
        const std::filesystem::path& mat_path,
        std::string variable_name = "Vfit");

    std::size_t available_frames() const override;
    double velocity_for_frame(std::size_t frame_index) const override;

private:
    VectorFrameVelocityProvider delegate_;
};

class JpgDirectoryFrameSink final : public FrameOutputSink {
public:
    JpgDirectoryFrameSink(
        std::filesystem::path output_dir,
        RenderConfig render_config);

    void write_frame(
        std::size_t frame_index,
        const MatrixXc& remapped_fliplr_image) override;

private:
    std::filesystem::path output_dir_;
    RenderConfig render_config_;
};

class OstreamLogger final : public Logger {
public:
    explicit OstreamLogger(std::ostream& stream);
    void log(const LogMessage& message) override;

private:
    std::ostream* stream_;
};

}  // namespace fft_imaging
