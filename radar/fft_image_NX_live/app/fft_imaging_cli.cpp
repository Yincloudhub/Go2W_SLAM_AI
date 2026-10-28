#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "fft_imaging/config.hpp"
#include "fft_imaging/matrix_summary.hpp"
#include "fft_imaging/profiler.hpp"
#include "fft_imaging/raw_capture_reader.hpp"
#include "fft_imaging/runtime.hpp"
#include "fft_imaging/tool_adapters.hpp"

namespace {

void print_usage() {
    std::cout
        << "Usage:\n"
        << "  fft_imaging_cli [--config config.txt] [--write-default-config out.txt] <raw_bin_path> <vfit_mat_path> <jpg_output_dir>\n\n"
        << "Behavior:\n"
        << "  Runs the first CPU C++ version of the MATLAB FFT instant imaging core chain,\n"
        << "  remaps to range-angle image space, and writes JPG frames.\n";
}

struct CliArgs {
    std::filesystem::path raw_path;
    std::filesystem::path vfit_mat_path;
    std::filesystem::path jpg_output_dir;
    std::filesystem::path config_path;
    std::filesystem::path write_default_config_path;
};

CliArgs parse_args(int argc, char** argv) {
    CliArgs args;
    std::vector<std::string> positional;

    for (int i = 1; i < argc; ++i) {
        const std::string token = argv[i];
        if (token == "--config") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--config requires a path argument.");
            }
            args.config_path = argv[++i];
            continue;
        }
        if (token == "--write-default-config") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--write-default-config requires a path argument.");
            }
            args.write_default_config_path = argv[++i];
            continue;
        }
        positional.push_back(token);
    }

    if (positional.empty()) {
        if (args.write_default_config_path.empty()) {
            throw std::invalid_argument("Expected positional arguments unless only writing a default config.");
        }
        return args;
    }

    if (positional.size() != 3) {
        throw std::invalid_argument("Expected 3 positional arguments.");
    }

    args.raw_path = positional[0];
    args.vfit_mat_path = positional[1];
    args.jpg_output_dir = positional[2];
    return args;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc < 2) {
            print_usage();
            return 1;
        }

        const CliArgs cli_args = parse_args(argc, argv);

        fft_imaging::AppConfig app_config = fft_imaging::make_matlab_equivalent_app_config();
        if (!cli_args.config_path.empty()) {
            app_config = fft_imaging::load_app_config_file(cli_args.config_path, app_config);
        }
        if (!cli_args.write_default_config_path.empty()) {
            fft_imaging::write_app_config_template(cli_args.write_default_config_path, app_config);
            if (cli_args.raw_path.empty()) {
                return 0;
            }
        }

        fft_imaging::ImagingConfig imaging_config;
        imaging_config.nr = app_config.fft_pipeline.nr;
        imaging_config.channel_selection = app_config.imaging.channel_selection;

        fft_imaging::Profiler profiler;
        fft_imaging::OstreamLogger logger(std::cout);
        fft_imaging::RawCaptureReader reader(app_config.acquisition.reader);
        fft_imaging::RawCaptureSession session = [&]() {
            auto stage = profiler.scoped(fft_imaging::StageKind::Read);
            return reader.load_session(cli_args.raw_path, imaging_config);
        }();
        fft_imaging::MatFileFrameVelocityProvider velocity_provider(cli_args.vfit_mat_path);
        fft_imaging::JpgDirectoryFrameSink output_sink(cli_args.jpg_output_dir, app_config.render);
        fft_imaging::ImagingRuntime runtime(app_config);

        std::cout << fft_imaging::describe_matrix(session.raw_channels(), "raw_channels") << "\n";
        std::cout << fft_imaging::describe_matrix(session.echo_matrix(), "echo_matrix") << "\n";
        const auto summary = runtime.run(
            session,
            velocity_provider,
            output_sink,
            &profiler,
            &logger);
        std::cout << "planned_frames: " << summary.frames_planned << "\n";
        std::cout << "backend: " << summary.backend_name << "\n";
        std::cout << fft_imaging::format_profiler_report(profiler) << "\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << "\n";
        print_usage();
        return 2;
    }
}
