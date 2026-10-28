#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <cstring>
#include <ctime>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#ifndef _WIN32
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#endif

#include "fft_imaging/backend.hpp"
#include "fft_imaging/config.hpp"
#include "fft_imaging/core_pipeline.hpp"
#include "fft_imaging/image_writer.hpp"
#include "fft_imaging/profiler.hpp"
#include "fft_imaging/realtime_stream.hpp"
#include "fft_imaging/tool_adapters.hpp"

namespace {

struct CliArgs {
    std::filesystem::path config_path;
    std::filesystem::path source_bin;
    std::filesystem::path vfit_mat_path;
    std::filesystem::path jpg_output_dir;
    std::filesystem::path video_output_path;
    std::filesystem::path metrics_log_path;
    double constant_velocity_mps = 1.0;
    double video_fps = 10.0;
    std::string dca_bind_ip = "192.168.33.30";
    int dca_data_port = 4098;
    int dca_timeout_ms = 2000;
    int dca_rcvbuf_bytes = 268435456;
    int mjpeg_port = 0;
    int mjpeg_quality = 85;
    bool timestamp_overlay = false;
    bool has_constant_velocity = false;
    bool source_dca1000 = false;
    std::size_t max_live_frames = 0;
};

void print_usage() {
    std::cout
        << "Usage:\n"
        << "  fft_imaging_live --config config.txt --source-bin radar_raw.bin [--vfit-mat Vfit.mat | --velocity-mps 1.0]\n"
        << "  fft_imaging_live --config config.txt --source-dca1000 [--dca-bind-ip 192.168.33.30] [--dca-data-port 4098]\n"
        << "                   [--jpg-output-dir out_dir] [--video-output out.avi] [--video-fps 10]\n"
        << "                   [--mjpeg-port 8090] [--mjpeg-quality 85]\n"
        << "                   [--metrics-log frames.csv]\n"
        << "                   [--timestamp-overlay]\n"
        << "                   [--max-live-frames N]\n\n"
        << "Behavior:\n"
        << "  Reads a chirp stream from a replay bin or DCA1000 UDP packets, forms sliding FFT imaging windows,\n"
        << "  remaps to range-angle space, and displays frames in an OpenCV window.\n";
}

CliArgs parse_args(int argc, char** argv) {
    CliArgs args;
    for (int i = 1; i < argc; ++i) {
        const std::string token = argv[i];
        auto require_value = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                throw std::invalid_argument(std::string(name) + " requires a value.");
            }
            return argv[++i];
        };

        if (token == "--config") {
            args.config_path = require_value("--config");
        } else if (token == "--source-bin") {
            args.source_bin = require_value("--source-bin");
        } else if (token == "--source-dca1000") {
            args.source_dca1000 = true;
        } else if (token == "--dca-bind-ip") {
            args.dca_bind_ip = require_value("--dca-bind-ip");
        } else if (token == "--dca-data-port") {
            args.dca_data_port = std::stoi(require_value("--dca-data-port"));
        } else if (token == "--dca-timeout-ms") {
            args.dca_timeout_ms = std::stoi(require_value("--dca-timeout-ms"));
        } else if (token == "--dca-rcvbuf-bytes") {
            args.dca_rcvbuf_bytes = std::stoi(require_value("--dca-rcvbuf-bytes"));
        } else if (token == "--mjpeg-port") {
            args.mjpeg_port = std::stoi(require_value("--mjpeg-port"));
        } else if (token == "--mjpeg-quality") {
            args.mjpeg_quality = std::stoi(require_value("--mjpeg-quality"));
        } else if (token == "--vfit-mat") {
            args.vfit_mat_path = require_value("--vfit-mat");
        } else if (token == "--velocity-mps") {
            args.constant_velocity_mps = std::stod(require_value("--velocity-mps"));
            args.has_constant_velocity = true;
        } else if (token == "--jpg-output-dir") {
            args.jpg_output_dir = require_value("--jpg-output-dir");
        } else if (token == "--video-output") {
            args.video_output_path = require_value("--video-output");
        } else if (token == "--video-fps") {
            args.video_fps = std::stod(require_value("--video-fps"));
        } else if (token == "--metrics-log") {
            args.metrics_log_path = require_value("--metrics-log");
        } else if (token == "--timestamp-overlay") {
            args.timestamp_overlay = true;
        } else if (token == "--max-live-frames") {
            args.max_live_frames = static_cast<std::size_t>(std::stoull(require_value("--max-live-frames")));
        } else if (token == "--help" || token == "-h") {
            print_usage();
            std::exit(0);
        } else {
            throw std::invalid_argument("Unknown argument: " + token);
        }
    }

    const bool has_source_bin = !args.source_bin.empty();
    if (has_source_bin == args.source_dca1000) {
        throw std::invalid_argument("Specify exactly one source: --source-bin radar_raw.bin or --source-dca1000.");
    }
    if (!args.video_output_path.empty() && args.video_fps <= 0.0) {
        throw std::invalid_argument("--video-fps must be positive.");
    }
    if (args.mjpeg_port < 0 || args.mjpeg_port > 65535) {
        throw std::invalid_argument("--mjpeg-port must be in [0, 65535].");
    }
    if (args.mjpeg_quality < 1 || args.mjpeg_quality > 100) {
        throw std::invalid_argument("--mjpeg-quality must be in [1, 100].");
    }
    return args;
}

class ConstantVelocityProvider final : public fft_imaging::FrameVelocityProvider {
public:
    explicit ConstantVelocityProvider(double velocity_mps) : velocity_mps_(velocity_mps) {}

    std::size_t available_frames() const override {
        return static_cast<std::size_t>(-1);
    }

    double velocity_for_frame(std::size_t) const override {
        return velocity_mps_;
    }

private:
    double velocity_mps_;
};

struct FrameSignalStats {
    double max_abs = 0.0;
    double mean_abs = 0.0;
    std::size_t finite_values = 0;
    std::size_t total_values = 0;
};

struct MjpegPublishStats {
    bool encoded = false;
    double encode_ms = 0.0;
    std::size_t jpeg_bytes = 0;
};

double elapsed_ms_since(std::chrono::steady_clock::time_point start) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

FrameSignalStats compute_frame_signal_stats(const fft_imaging::MatrixXc& image) {
    FrameSignalStats stats;
    stats.total_values = static_cast<std::size_t>(image.size());
    double sum_abs = 0.0;
    for (Eigen::Index row = 0; row < image.rows(); ++row) {
        for (Eigen::Index col = 0; col < image.cols(); ++col) {
            const auto value = image(row, col);
            if (!std::isfinite(value.real()) || !std::isfinite(value.imag())) {
                continue;
            }
            const double mag = std::abs(value);
            sum_abs += mag;
            stats.max_abs = std::max(stats.max_abs, mag);
            ++stats.finite_values;
        }
    }
    if (stats.finite_values > 0) {
        stats.mean_abs = sum_abs / static_cast<double>(stats.finite_values);
    }
    return stats;
}

bool has_invalid_live_frame_signal(
    const FrameSignalStats& stats,
    std::string& reason) {
    if (stats.total_values == 0 || stats.finite_values == 0) {
        reason = "no finite pixels";
        return true;
    }
    if (stats.finite_values != stats.total_values) {
        reason = "non-finite pixels";
        return true;
    }
    if (stats.max_abs <= 1e-12) {
        reason = "near-zero peak";
        return true;
    }
    return false;
}

std::string wall_clock_timestamp_text() {
    const auto now = std::chrono::system_clock::now();
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S")
        << "." << std::setw(3) << std::setfill('0') << ms.count();
    return oss.str();
}

void draw_timestamp_overlay(cv::Mat& image) {
    const std::string text = wall_clock_timestamp_text();
    const int font = cv::FONT_HERSHEY_SIMPLEX;
    const double scale = 0.72;
    const int thickness = 2;
    int baseline = 0;
    const cv::Size text_size = cv::getTextSize(text, font, scale, thickness, &baseline);
    const cv::Point origin(18, 34);
    const cv::Rect background(
        origin.x - 8,
        origin.y - text_size.height - 8,
        text_size.width + 16,
        text_size.height + baseline + 16);
    cv::rectangle(image, background, cv::Scalar(20, 35, 45), cv::FILLED);
    cv::putText(image, text, origin, font, scale, cv::Scalar(255, 255, 255), thickness, cv::LINE_AA);
}

std::unique_ptr<fft_imaging::FrameVelocityProvider> make_velocity_provider(const CliArgs& args) {
    if (!args.vfit_mat_path.empty()) {
        return std::make_unique<fft_imaging::MatFileFrameVelocityProvider>(args.vfit_mat_path);
    }
    if (!args.has_constant_velocity) {
        std::cout << "[WARN][live] no --vfit-mat or --velocity-mps provided; using constant velocity 1.0 m/s\n";
    }
    return std::make_unique<ConstantVelocityProvider>(args.constant_velocity_mps);
}

class MjpegServer {
public:
    MjpegServer(int port, int jpeg_quality) : port_(port), jpeg_quality_(jpeg_quality) {}

    ~MjpegServer() {
        stop();
    }

    bool enabled() const {
        return port_ > 0;
    }

    void start() {
        if (!enabled()) {
            return;
        }
#ifdef _WIN32
        throw std::runtime_error("MJPEG server is only supported on POSIX/Linux builds.");
#else
        server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd_ < 0) {
            throw std::runtime_error("Failed to create MJPEG server socket.");
        }

        const int opt = 1;
        ::setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_ANY);
        address.sin_port = htons(static_cast<uint16_t>(port_));
        if (::bind(server_fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
            const std::string error = std::strerror(errno);
            close_server_socket();
            throw std::runtime_error("Failed to bind MJPEG server on port " + std::to_string(port_) + ": " + error);
        }
        if (::listen(server_fd_, 8) < 0) {
            const std::string error = std::strerror(errno);
            close_server_socket();
            throw std::runtime_error("Failed to listen on MJPEG server socket: " + error);
        }

        accept_thread_ = std::thread([this]() { accept_loop(); });
        std::cout << "[INFO][mjpeg] serving http://0.0.0.0:" << port_ << "/radar.mjpg\n";
#endif
    }

    MjpegPublishStats publish(const cv::Mat& image) {
        MjpegPublishStats stats;
        if (!enabled()) {
            return stats;
        }
        std::vector<uchar> jpeg;
        const std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
        const auto encode_start = std::chrono::steady_clock::now();
        if (!cv::imencode(".jpg", image, jpeg, params)) {
            std::cerr << "[WARN][mjpeg] JPEG encode failed; frame skipped\n";
            return stats;
        }
        stats.encode_ms = elapsed_ms_since(encode_start);
        stats.jpeg_bytes = jpeg.size();
        stats.encoded = true;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_jpeg_ = std::move(jpeg);
            ++frame_id_;
        }
        cv_.notify_all();
        return stats;
    }

    void stop() {
        if (stopped_) {
            return;
        }
        stopped_ = true;
        stop_requested_.store(true);
        cv_.notify_all();
#ifndef _WIN32
        close_server_socket();
#endif
        if (accept_thread_.joinable()) {
            accept_thread_.join();
        }
        {
            std::lock_guard<std::mutex> lock(client_threads_mutex_);
            for (auto& thread : client_threads_) {
                if (thread.joinable()) {
                    thread.join();
                }
            }
            client_threads_.clear();
        }
    }

private:
    static bool send_all(int fd, const void* data, std::size_t size) {
#ifdef _WIN32
        (void)fd;
        (void)data;
        (void)size;
        return false;
#else
        const char* cursor = static_cast<const char*>(data);
        std::size_t remaining = size;
        while (remaining > 0) {
            const ssize_t sent = ::send(fd, cursor, remaining, MSG_NOSIGNAL);
            if (sent <= 0) {
                return false;
            }
            cursor += sent;
            remaining -= static_cast<std::size_t>(sent);
        }
        return true;
#endif
    }

    static bool send_text(int fd, const std::string& text) {
        return send_all(fd, text.data(), text.size());
    }

#ifndef _WIN32
    void close_server_socket() {
        if (server_fd_ >= 0) {
            ::shutdown(server_fd_, SHUT_RDWR);
            ::close(server_fd_);
            server_fd_ = -1;
        }
    }

    void accept_loop() {
        while (!stop_requested_.load()) {
            sockaddr_in client_address{};
            socklen_t client_length = sizeof(client_address);
            const int client_fd = ::accept(server_fd_, reinterpret_cast<sockaddr*>(&client_address), &client_length);
            if (client_fd < 0) {
                if (!stop_requested_.load()) {
                    std::cerr << "[WARN][mjpeg] accept failed: " << std::strerror(errno) << "\n";
                }
                continue;
            }
            std::lock_guard<std::mutex> lock(client_threads_mutex_);
            client_threads_.emplace_back(&MjpegServer::client_loop, this, client_fd);
        }
    }

    void client_loop(int client_fd) {
        timeval send_timeout{};
        send_timeout.tv_sec = 1;
        send_timeout.tv_usec = 0;
        ::setsockopt(client_fd, SOL_SOCKET, SO_SNDTIMEO, &send_timeout, sizeof(send_timeout));

        const std::string header =
            "HTTP/1.0 200 OK\r\n"
            "Server: fft_imaging_live\r\n"
            "Cache-Control: no-cache\r\n"
            "Pragma: no-cache\r\n"
            "Connection: close\r\n"
            "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
        if (!send_text(client_fd, header)) {
            ::close(client_fd);
            return;
        }

        std::uint64_t last_frame_id = 0;
        while (!stop_requested_.load()) {
            std::vector<uchar> jpeg;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [&]() {
                    return stop_requested_.load() || frame_id_ != last_frame_id;
                });
                if (stop_requested_.load()) {
                    break;
                }
                jpeg = latest_jpeg_;
                last_frame_id = frame_id_;
            }
            if (jpeg.empty()) {
                continue;
            }
            const std::string part_header =
                "--frame\r\n"
                "Content-Type: image/jpeg\r\n"
                "Content-Length: " + std::to_string(jpeg.size()) + "\r\n\r\n";
            if (!send_text(client_fd, part_header) ||
                !send_all(client_fd, jpeg.data(), jpeg.size()) ||
                !send_text(client_fd, "\r\n")) {
                break;
            }
        }
        ::close(client_fd);
    }
#endif

    int port_ = 0;
    int jpeg_quality_ = 85;
    std::atomic<bool> stop_requested_{false};
    bool stopped_ = false;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::vector<uchar> latest_jpeg_;
    std::uint64_t frame_id_ = 0;
    std::thread accept_thread_;
    std::mutex client_threads_mutex_;
    std::vector<std::thread> client_threads_;
#ifndef _WIN32
    int server_fd_ = -1;
#endif
};

}  // namespace

int main(int argc, char** argv) {
    try {
        const CliArgs cli_args = parse_args(argc, argv);

        fft_imaging::AppConfig app_config = fft_imaging::make_matlab_equivalent_app_config();
        if (!cli_args.config_path.empty()) {
            app_config = fft_imaging::load_app_config_file(cli_args.config_path, app_config);
        }
        if (app_config.fft_pipeline.frame_selection.frag == 0 ||
            app_config.fft_pipeline.frame_selection.step == 0) {
            throw std::runtime_error("fft.frag and fft.step must be positive for live imaging.");
        }
        if (app_config.stream.ring_columns < app_config.fft_pipeline.frame_selection.frag) {
            throw std::runtime_error("stream.ring_columns must be >= fft.frag.");
        }

        fft_imaging::ImagingConfig imaging_config;
        imaging_config.nr = app_config.fft_pipeline.nr;
        imaging_config.channel_selection = app_config.imaging.channel_selection;

        std::unique_ptr<fft_imaging::ComplexColumnSource> source;
        std::size_t bytes_per_chirp = 0;
        std::string source_label;
        bool replay_timing_enabled = false;
        if (!cli_args.source_bin.empty()) {
            auto replay_source = std::make_unique<fft_imaging::BinReplaySource>(
                cli_args.source_bin,
                app_config.acquisition.reader,
                imaging_config);
            bytes_per_chirp = replay_source->bytes_per_column();
            source_label = cli_args.source_bin.string();
            replay_timing_enabled = true;
            source = std::move(replay_source);
        } else {
            auto dca_source = std::make_unique<fft_imaging::Dca1000ColumnSource>(
                cli_args.dca_bind_ip,
                cli_args.dca_data_port,
                app_config.acquisition.reader,
                imaging_config,
                cli_args.dca_timeout_ms,
                cli_args.dca_rcvbuf_bytes);
            bytes_per_chirp = dca_source->bytes_per_column();
            source_label = "dca1000://" + cli_args.dca_bind_ip + ":" + std::to_string(cli_args.dca_data_port);
            source = std::move(dca_source);
        }
        fft_imaging::EchoRingBuffer ring(app_config.fft_pipeline.nr, app_config.stream.ring_columns);
        fft_imaging::PipelineConfig pipeline_config;
        pipeline_config.mode = fft_imaging::PipelineMode::Runtime;
        fft_imaging::CorePipeline pipeline(imaging_config, pipeline_config);
        const auto backend = fft_imaging::make_imaging_backend(app_config.imaging.backend);
        auto velocity_provider = make_velocity_provider(cli_args);

        fft_imaging::RangeAngleRemapPlan remap_plan;
        fft_imaging::RemapBuffers remap_buffers;
        bool has_remap_plan = false;
        fft_imaging::MatrixXc window;
        cv::VideoWriter video_writer;
        MjpegServer mjpeg_server(cli_args.mjpeg_port, cli_args.mjpeg_quality);
        mjpeg_server.start();
        std::ofstream metrics_log;
        const auto live_start_time = std::chrono::steady_clock::now();
        if (!cli_args.metrics_log_path.empty()) {
            if (!cli_args.metrics_log_path.parent_path().empty()) {
                std::filesystem::create_directories(cli_args.metrics_log_path.parent_path());
            }
            metrics_log.open(cli_args.metrics_log_path);
            if (!metrics_log) {
                throw std::runtime_error("Failed to open metrics log: " + cli_args.metrics_log_path.string());
            }
            metrics_log
                << "runtime_s,frame_index,columns_snapshot,source_columns_read,ring_size_columns,"
                << "v_mps,range_fft_ms,azimuth_fft_ms,remap_ms,render_ms,mjpeg_encode_ms,"
                << "jpeg_bytes,video_write_ms,window_max_abs,window_mean_abs,remap_max_abs,"
                << "remap_mean_abs,remap_finite_values,remap_total_values,invalid_frame\n";
            metrics_log << std::fixed << std::setprecision(6);
            std::cout << "[INFO][live] metrics_log=" << cli_args.metrics_log_path << "\n";
        }
        std::size_t frame_index = 0;
        std::size_t invalid_live_output_frames = 0;

        if (app_config.display.enabled) {
            cv::namedWindow("TI Radar FFT Imaging Live", cv::WINDOW_NORMAL);
        }

        const auto column_period = replay_timing_enabled && app_config.stream.replay_rate_hz > 0.0
            ? std::chrono::duration<double>(1.0 / app_config.stream.replay_rate_hz)
            : std::chrono::duration<double>(0.0);
        auto next_column_time = std::chrono::steady_clock::now();

        std::cout << "[INFO][live] source=" << source_label
                  << ", bytes_per_chirp=" << bytes_per_chirp
                  << ", nr=" << app_config.fft_pipeline.nr
                  << ", frag=" << app_config.fft_pipeline.frame_selection.frag
                  << ", step=" << app_config.fft_pipeline.frame_selection.step
                  << ", ring_columns=" << app_config.stream.ring_columns
                  << "\n";

        std::mutex ring_mutex;
        std::condition_variable ring_cv;
        std::atomic<bool> stop_requested{false};
        std::atomic<bool> source_done{false};
        std::exception_ptr reader_error;

        std::thread reader_thread([&]() {
            try {
                fft_imaging::VectorXc column;
                while (!stop_requested.load() && source->next_column(column)) {
                    {
                        std::lock_guard<std::mutex> lock(ring_mutex);
                        ring.push_column(column);
                    }
                    ring_cv.notify_one();

                    if (column_period.count() > 0.0) {
                        next_column_time += std::chrono::duration_cast<std::chrono::steady_clock::duration>(column_period);
                        std::this_thread::sleep_until(next_column_time);
                    }
                }
            } catch (...) {
                std::lock_guard<std::mutex> lock(ring_mutex);
                reader_error = std::current_exception();
            }
            source_done.store(true);
            ring_cv.notify_one();
        });

        const std::size_t frag = app_config.fft_pipeline.frame_selection.frag;
        const std::size_t step = app_config.fft_pipeline.frame_selection.step;
        std::size_t next_frame_total_columns = frag;
        std::exception_ptr fatal_reader_error;

        while (!stop_requested.load()) {
            std::size_t columns_snapshot = 0;
            {
                std::unique_lock<std::mutex> lock(ring_mutex);
                ring_cv.wait(lock, [&]() {
                    return stop_requested.load() ||
                           source_done.load() ||
                           ring.total_pushed() >= next_frame_total_columns;
                });
                if (stop_requested.load()) {
                    break;
                }
                if (reader_error) {
                    if (frame_index == 0) {
                        fatal_reader_error = reader_error;
                        break;
                    }
                    try {
                        std::rethrow_exception(reader_error);
                    } catch (const std::exception& ex) {
                        std::cerr << "[WARN][live] stopping after reader error: " << ex.what() << "\n";
                    }
                    break;
                }
                if (source_done.load() && ring.total_pushed() < next_frame_total_columns) {
                    break;
                }
                if (!ring.latest_window(frag, window)) {
                    continue;
                }
                columns_snapshot = ring.total_pushed();
            }
            next_frame_total_columns = columns_snapshot + step;

            fft_imaging::FrameSelection frame_selection = app_config.fft_pipeline.frame_selection;
            frame_selection.frame_index = frame_index;
            const auto artifacts = pipeline.run_fft_instant_imaging_frame(window, frame_selection);
            const double v_mps = velocity_provider->velocity_for_frame(frame_index);
            if (!has_remap_plan || std::abs(remap_plan.v_mps - v_mps) > 1e-12) {
                remap_plan = fft_imaging::build_range_angle_remap_plan(
                    artifacts.fft2_after_azimuth_fft.rows(),
                    artifacts.fft2_after_azimuth_fft.cols(),
                    app_config.imaging.scene_grid,
                    v_mps);
                fft_imaging::resize_remap_buffers(remap_plan, remap_buffers);
                has_remap_plan = true;
            }
            const auto remap_start = std::chrono::steady_clock::now();
            backend->remap_to_range_angle(
                artifacts.fft2_after_azimuth_fft,
                remap_plan,
                app_config.imaging.interpolation_mode,
                remap_buffers);
            const double remap_ms = elapsed_ms_since(remap_start);

            if (app_config.display.save_every_n > 0 &&
                !cli_args.jpg_output_dir.empty() &&
                frame_index % app_config.display.save_every_n == 0) {
                fft_imaging::JpgDirectoryFrameSink sink(cli_args.jpg_output_dir, app_config.render);
                sink.write_frame(frame_index, remap_buffers.remapped_fliplr_image);
            }

            if (app_config.display.enabled || !cli_args.video_output_path.empty() ||
                mjpeg_server.enabled() || metrics_log) {
                const FrameSignalStats window_stats = compute_frame_signal_stats(window);
                const FrameSignalStats remap_stats = compute_frame_signal_stats(remap_buffers.remapped_fliplr_image);
                std::string invalid_reason;
                const bool invalid_frame = has_invalid_live_frame_signal(remap_stats, invalid_reason);
                const auto render_start = std::chrono::steady_clock::now();
                cv::Mat image = fft_imaging::render_complex_image_to_bgr(
                    remap_buffers.remapped_fliplr_image,
                    app_config.render);
                if (cli_args.timestamp_overlay) {
                    draw_timestamp_overlay(image);
                }
                const double render_ms = elapsed_ms_since(render_start);
                if (invalid_frame) {
                    ++invalid_live_output_frames;
                    if (invalid_live_output_frames <= 5 || invalid_live_output_frames % 25 == 0) {
                        std::cerr << "[WARN][live] output frame has weak/invalid signal; still publishing frame="
                                  << frame_index
                                  << ", reason=" << invalid_reason
                                  << ", window_max_abs=" << window_stats.max_abs
                                  << ", remap_max_abs=" << remap_stats.max_abs
                                  << ", invalid_count=" << invalid_live_output_frames
                                  << "\n";
                    }
                }
                if (frame_index % 25 == 0) {
                    std::cout << "[INFO][live] signal frame=" << frame_index
                              << ", window_max_abs=" << window_stats.max_abs
                              << ", window_mean_abs=" << window_stats.mean_abs
                              << ", remap_max_abs=" << remap_stats.max_abs
                              << ", remap_mean_abs=" << remap_stats.mean_abs
                              << "\n";
                }
                MjpegPublishStats mjpeg_stats;
                if (mjpeg_server.enabled()) {
                    mjpeg_stats = mjpeg_server.publish(image);
                }
                double video_write_ms = 0.0;
                if (!cli_args.video_output_path.empty()) {
                    if (!video_writer.isOpened()) {
                        if (!cli_args.video_output_path.parent_path().empty()) {
                            std::filesystem::create_directories(cli_args.video_output_path.parent_path());
                        }
                        const int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
                        video_writer.open(
                            cli_args.video_output_path.string(),
                            fourcc,
                            cli_args.video_fps,
                            image.size(),
                            true);
                        if (!video_writer.isOpened()) {
                            throw std::runtime_error(
                                "Failed to open video output: " + cli_args.video_output_path.string());
                        }
                    }
                    const auto video_write_start = std::chrono::steady_clock::now();
                    video_writer.write(image);
                    video_write_ms = elapsed_ms_since(video_write_start);
                }
                if (metrics_log) {
                    metrics_log
                        << elapsed_ms_since(live_start_time) / 1000.0 << ','
                        << frame_index << ','
                        << columns_snapshot << ','
                        << source->columns_read() << ','
                        << ring.size() << ','
                        << v_mps << ','
                        << artifacts.timings.range_fft_ms << ','
                        << artifacts.timings.azimuth_fft_ms << ','
                        << remap_ms << ','
                        << render_ms << ','
                        << mjpeg_stats.encode_ms << ','
                        << mjpeg_stats.jpeg_bytes << ','
                        << video_write_ms << ','
                        << window_stats.max_abs << ','
                        << window_stats.mean_abs << ','
                        << remap_stats.max_abs << ','
                        << remap_stats.mean_abs << ','
                        << remap_stats.finite_values << ','
                        << remap_stats.total_values << ','
                        << (invalid_frame ? 1 : 0)
                        << '\n';
                }
                if (app_config.display.enabled) {
                    cv::imshow("TI Radar FFT Imaging Live", image);
                    const int key = cv::waitKey(1);
                    if (key == 'q' || key == 'Q' || key == 27) {
                        stop_requested.store(true);
                        ring_cv.notify_one();
                        break;
                    }
                }
            }

            std::cout << "[INFO][live] frame=" << frame_index
                      << ", columns=" << columns_snapshot
                      << ", v_mps=" << v_mps
                      << ", range_fft_ms=" << artifacts.timings.range_fft_ms
                      << ", azimuth_fft_ms=" << artifacts.timings.azimuth_fft_ms
                      << "\n";
            ++frame_index;
            if (cli_args.max_live_frames > 0 && frame_index >= cli_args.max_live_frames) {
                stop_requested.store(true);
                ring_cv.notify_one();
                break;
            }
        }
        stop_requested.store(true);
        ring_cv.notify_one();
        if (reader_thread.joinable()) {
            reader_thread.join();
        }
        if (video_writer.isOpened()) {
            video_writer.release();
        }
        if (fatal_reader_error) {
            std::rethrow_exception(fatal_reader_error);
        }

        std::cout << "[INFO][live] completed: frames=" << frame_index
                  << ", chirps=" << source->columns_read() << "\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << "\n";
        print_usage();
        return 2;
    }
}
