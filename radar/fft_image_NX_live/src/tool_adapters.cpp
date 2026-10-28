#include "fft_imaging/tool_adapters.hpp"

#include <iomanip>
#include <ostream>
#include <sstream>
#include <stdexcept>

namespace fft_imaging {

VectorFrameVelocityProvider::VectorFrameVelocityProvider(VectorXd velocities_mps)
    : velocities_mps_(std::move(velocities_mps)) {}

std::size_t VectorFrameVelocityProvider::available_frames() const {
    return static_cast<std::size_t>(velocities_mps_.size());
}

double VectorFrameVelocityProvider::velocity_for_frame(std::size_t frame_index) const {
    if (frame_index >= available_frames()) {
        throw std::out_of_range("Requested frame velocity is out of range.");
    }
    return velocities_mps_(static_cast<Eigen::Index>(frame_index));
}

MatFileFrameVelocityProvider::MatFileFrameVelocityProvider(
    const std::filesystem::path& mat_path,
    std::string variable_name)
    : delegate_(read_vfit_mat(mat_path, variable_name)) {}

std::size_t MatFileFrameVelocityProvider::available_frames() const {
    return delegate_.available_frames();
}

double MatFileFrameVelocityProvider::velocity_for_frame(std::size_t frame_index) const {
    return delegate_.velocity_for_frame(frame_index);
}

JpgDirectoryFrameSink::JpgDirectoryFrameSink(
    std::filesystem::path output_dir,
    RenderConfig render_config)
    : output_dir_(std::move(output_dir)),
      render_config_(render_config) {}

void JpgDirectoryFrameSink::write_frame(
    std::size_t frame_index,
    const MatrixXc& remapped_fliplr_image) {
    std::ostringstream file_name;
    file_name << std::setw(4) << std::setfill('0') << (frame_index + 1) << ".jpg";
    write_jpg_from_complex_image(
        remapped_fliplr_image,
        output_dir_ / file_name.str(),
        render_config_);
}

OstreamLogger::OstreamLogger(std::ostream& stream) : stream_(&stream) {}

void OstreamLogger::log(const LogMessage& message) {
    (*stream_) << "[" << log_level_name(message.level) << "]";
    if (!message.component.empty()) {
        (*stream_) << "[" << message.component << "]";
    }
    (*stream_) << " " << message.text << "\n";
}

}  // namespace fft_imaging
