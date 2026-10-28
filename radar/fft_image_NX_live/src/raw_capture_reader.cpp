#include "fft_imaging/raw_capture_reader.hpp"

#include <fstream>
#include <stdexcept>
#include <vector>

namespace fft_imaging {

namespace {

std::vector<int16_t> read_int16_file(const std::filesystem::path& file_path) {
    std::ifstream input(file_path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("Failed to open raw capture file: " + file_path.string());
    }

    input.seekg(0, std::ios::end);
    const auto size_bytes = input.tellg();
    input.seekg(0, std::ios::beg);

    if (size_bytes < 0 || (size_bytes % static_cast<std::streamoff>(sizeof(int16_t))) != 0) {
        throw std::runtime_error("Raw capture file size is not aligned to int16 samples.");
    }

    std::vector<int16_t> samples(static_cast<std::size_t>(size_bytes) / sizeof(int16_t));
    input.read(reinterpret_cast<char*>(samples.data()), static_cast<std::streamsize>(size_bytes));
    if (!input) {
        throw std::runtime_error("Failed to read all samples from raw capture file.");
    }

    return samples;
}

}  // namespace

RawCaptureReader::RawCaptureReader(ReaderConfig config) : config_(config) {}

MatrixXc RawCaptureReader::read_dca1000_complex(const std::filesystem::path& file_path) const {
    auto samples = read_int16_file(file_path);

    if (config_.num_adc_bits != 16) {
        const int l_max = (1 << (config_.num_adc_bits - 1)) - 1;
        const int wrap = 1 << config_.num_adc_bits;
        for (auto& sample : samples) {
            if (sample > l_max) {
                sample = static_cast<int16_t>(sample - wrap);
            }
        }
    }

    if (config_.is_real) {
        if (samples.size() % static_cast<std::size_t>(config_.num_lanes) != 0) {
            throw std::runtime_error("Real-mode sample count is not divisible by num_lanes.");
        }

        const Eigen::Index cols =
            static_cast<Eigen::Index>(samples.size() / static_cast<std::size_t>(config_.num_lanes));
        MatrixXc output(config_.num_lanes, cols);
        for (Eigen::Index col = 0; col < cols; ++col) {
            for (int lane = 0; lane < config_.num_lanes; ++lane) {
                const auto index = static_cast<std::size_t>(col) * config_.num_lanes + lane;
                output(lane, col) = Complex(static_cast<double>(samples[index]), 0.0);
            }
        }
        return output;
    }

    const int packed_rows = config_.num_lanes * 2;
    if (samples.size() % static_cast<std::size_t>(packed_rows) != 0) {
        throw std::runtime_error("Complex-mode sample count is not divisible by num_lanes*2.");
    }

    const Eigen::Index cols =
        static_cast<Eigen::Index>(samples.size() / static_cast<std::size_t>(packed_rows));
    MatrixXc output(config_.num_lanes, cols);

    // MATLAB reference:
    // adcData = reshape(adcData, numLanes*2, []);
    // adcData = adcData([1,2,3,4],:) + 1j * adcData([5,6,7,8],:)
    for (Eigen::Index col = 0; col < cols; ++col) {
        for (int lane = 0; lane < config_.num_lanes; ++lane) {
            const auto real_index = static_cast<std::size_t>(col) * packed_rows + lane;
            const auto imag_index = static_cast<std::size_t>(col) * packed_rows + config_.num_lanes + lane;
            output(lane, col) = Complex(
                static_cast<double>(samples[real_index]),
                static_cast<double>(samples[imag_index]));
        }
    }

    return output;
}

RawCaptureSession RawCaptureReader::load_session(
    const std::filesystem::path& file_path,
    const ImagingConfig& imaging_config) const {
    return make_raw_capture_session(
        read_dca1000_complex(file_path),
        imaging_config,
        file_path);
}

}  // namespace fft_imaging
