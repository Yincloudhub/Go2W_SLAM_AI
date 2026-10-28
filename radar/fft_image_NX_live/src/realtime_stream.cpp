#include "fft_imaging/realtime_stream.hpp"

#include <algorithm>
#include <cstring>
#include <stdexcept>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "fft_imaging/matrix_ops.hpp"

namespace fft_imaging {

namespace {

std::size_t checked_packed_rows(const ReaderConfig& config) {
    if (config.num_lanes <= 0) {
        throw std::invalid_argument("ReaderConfig.num_lanes must be positive.");
    }
    return static_cast<std::size_t>(config.num_lanes) * (config.is_real ? 1U : 2U);
}

Complex make_sample(const std::vector<int16_t>& samples, std::size_t base, const ReaderConfig& config, int lane) {
    if (config.is_real) {
        return Complex(static_cast<double>(samples[base + static_cast<std::size_t>(lane)]), 0.0);
    }
    return Complex(
        static_cast<double>(samples[base + static_cast<std::size_t>(lane)]),
        static_cast<double>(samples[base + static_cast<std::size_t>(config.num_lanes + lane)]));
}

std::uint32_t read_le_u32(const uint8_t* data) {
    return static_cast<std::uint32_t>(data[0]) |
           (static_cast<std::uint32_t>(data[1]) << 8U) |
           (static_cast<std::uint32_t>(data[2]) << 16U) |
           (static_cast<std::uint32_t>(data[3]) << 24U);
}

}  // namespace

BinReplaySource::BinReplaySource(
    std::filesystem::path bin_path,
    ReaderConfig reader_config,
    ImagingConfig imaging_config)
    : bin_path_(std::move(bin_path)),
      reader_config_(reader_config),
      imaging_config_(std::move(imaging_config)),
      input_(bin_path_, std::ios::binary) {
    if (!input_) {
        throw std::runtime_error("Failed to open bin replay source: " + bin_path_.string());
    }
    if (imaging_config_.nr == 0) {
        throw std::invalid_argument("ImagingConfig.nr must be positive.");
    }
    if (imaging_config_.channel_selection.mode != ChannelSelectionMode::Single) {
        throw std::invalid_argument("BinReplaySource currently supports single channel selection only.");
    }
    if (imaging_config_.channel_selection.channel_index < 0 ||
        imaging_config_.channel_selection.channel_index >= reader_config_.num_lanes) {
        throw std::invalid_argument("Selected channel index is outside configured lane count.");
    }
    sample_buffer_.resize(
        static_cast<std::size_t>(imaging_config_.nr) * checked_packed_rows(reader_config_));
}

bool BinReplaySource::next_column(VectorXc& column) {
    input_.read(
        reinterpret_cast<char*>(sample_buffer_.data()),
        static_cast<std::streamsize>(sample_buffer_.size() * sizeof(int16_t)));
    if (input_.gcount() == 0 && input_.eof()) {
        return false;
    }
    if (input_.gcount() != static_cast<std::streamsize>(sample_buffer_.size() * sizeof(int16_t))) {
        throw std::runtime_error("Bin replay ended with an incomplete chirp column.");
    }

    const auto nr = static_cast<Eigen::Index>(imaging_config_.nr);
    column.resize(nr);
    const std::size_t packed_rows = checked_packed_rows(reader_config_);
    const int lane = imaging_config_.channel_selection.channel_index;
    for (Eigen::Index row = 0; row < nr; ++row) {
        const std::size_t base = static_cast<std::size_t>(row) * packed_rows;
        column(row) = make_sample(sample_buffer_, base, reader_config_, lane);
    }
    ++columns_read_;
    return true;
}

std::size_t BinReplaySource::columns_read() const {
    return columns_read_;
}

std::size_t BinReplaySource::bytes_per_column() const {
    return sample_buffer_.size() * sizeof(int16_t);
}

std::size_t Dca1000ColumnSource::bytes_per_column() const {
    return static_cast<std::size_t>(imaging_config_.nr) *
           checked_packed_rows(reader_config_) *
           sizeof(int16_t);
}

Dca1000ColumnSource::Dca1000ColumnSource(
    std::string bind_ip,
    int data_port,
    ReaderConfig reader_config,
    ImagingConfig imaging_config,
    int timeout_ms,
    int socket_receive_buffer_bytes)
    : bind_ip_(std::move(bind_ip)),
      data_port_(data_port),
      reader_config_(reader_config),
      imaging_config_(std::move(imaging_config)),
      timeout_ms_(timeout_ms),
      socket_receive_buffer_bytes_(socket_receive_buffer_bytes),
      chirp_bytes_(bytes_per_column()),
      packet_buffer_(1466) {
    if (imaging_config_.nr == 0) {
        throw std::invalid_argument("ImagingConfig.nr must be positive.");
    }
    if (imaging_config_.channel_selection.mode != ChannelSelectionMode::Single) {
        throw std::invalid_argument("Dca1000ColumnSource currently supports single channel selection only.");
    }
    if (imaging_config_.channel_selection.channel_index < 0 ||
        imaging_config_.channel_selection.channel_index >= reader_config_.num_lanes) {
        throw std::invalid_argument("Selected channel index is outside configured lane count.");
    }

    socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        throw std::runtime_error("Failed to create DCA1000 UDP socket.");
    }
    if (socket_receive_buffer_bytes_ > 0) {
        ::setsockopt(
            socket_fd_,
            SOL_SOCKET,
            SO_RCVBUF,
            &socket_receive_buffer_bytes_,
            sizeof(socket_receive_buffer_bytes_));
    }
    timeval tv{};
    tv.tv_sec = timeout_ms_ / 1000;
    tv.tv_usec = (timeout_ms_ % 1000) * 1000;
    ::setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(data_port_));
    if (::inet_pton(AF_INET, bind_ip_.c_str(), &addr.sin_addr) != 1) {
        ::close(socket_fd_);
        socket_fd_ = -1;
        throw std::runtime_error("Invalid DCA1000 bind IP: " + bind_ip_);
    }
    if (::bind(socket_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
        throw std::runtime_error("Failed to bind DCA1000 UDP socket on " + bind_ip_ + ":" + std::to_string(data_port_));
    }
}

Dca1000ColumnSource::~Dca1000ColumnSource() {
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
    }
}

bool Dca1000ColumnSource::next_column(VectorXc& column) {
    while (!pop_column(column)) {
        if (!recv_payload()) {
            return false;
        }
    }
    return true;
}

std::size_t Dca1000ColumnSource::columns_read() const {
    return columns_read_;
}

std::size_t Dca1000ColumnSource::packets_received() const {
    return packets_received_;
}

std::size_t Dca1000ColumnSource::sequence_gaps() const {
    return sequence_gaps_;
}

bool Dca1000ColumnSource::recv_payload() {
    sockaddr_in src{};
    socklen_t src_len = sizeof(src);
    const ssize_t n = ::recvfrom(
        socket_fd_,
        reinterpret_cast<char*>(packet_buffer_.data()),
        packet_buffer_.size(),
        0,
        reinterpret_cast<sockaddr*>(&src),
        &src_len);
    if (n < 0) {
        throw std::runtime_error("DCA1000 UDP receive timed out.");
    }
    if (n < 11) {
        return true;
    }
    ++packets_received_;
    const std::uint32_t seq = read_le_u32(packet_buffer_.data());
    if (seq == 0) {
        return true;
    }

    constexpr std::size_t kHeaderBytes = 10;
    constexpr std::size_t kPayloadBytes = 1456;
    const std::size_t payload_size = std::min<std::size_t>(static_cast<std::size_t>(n) - kHeaderBytes, kPayloadBytes);
    const std::uint64_t packet_data_start = static_cast<std::uint64_t>(seq - 1U) * kPayloadBytes;
    const uint8_t* payload = packet_buffer_.data() + kHeaderBytes;
    std::size_t payload_offset = 0;

    if (!aligned_) {
        const std::uint64_t chirp_offset = packet_data_start % bytes_per_column();
        const std::uint64_t skip = (bytes_per_column() - chirp_offset) % bytes_per_column();
        if (skip >= payload_size) {
            return true;
        }
        payload_offset = static_cast<std::size_t>(skip);
        expected_next_data_start_ = packet_data_start + payload_offset;
        aligned_ = true;
    }

    const std::uint64_t actual_start = packet_data_start + payload_offset;
    if (actual_start > expected_next_data_start_) {
        const auto gap = static_cast<std::size_t>(actual_start - expected_next_data_start_);
        ++sequence_gaps_;
        if (gap > bytes_per_column()) {
            fifo_.clear();
            aligned_ = false;
            return true;
        }
        fifo_.insert(fifo_.end(), gap, 0);
    } else if (actual_start < expected_next_data_start_) {
        const auto overlap = static_cast<std::size_t>(expected_next_data_start_ - actual_start);
        if (overlap >= payload_size - payload_offset) {
            return true;
        }
        payload_offset += overlap;
    }

    append_payload_bytes(payload + payload_offset, payload_size - payload_offset);
    expected_next_data_start_ = packet_data_start + payload_size;
    return true;
}

void Dca1000ColumnSource::append_payload_bytes(const uint8_t* data, std::size_t size) {
    fifo_.insert(fifo_.end(), data, data + size);
}

bool Dca1000ColumnSource::pop_column(VectorXc& column) {
    const std::size_t column_bytes = bytes_per_column();
    if (fifo_.size() < column_bytes) {
        return false;
    }
    std::copy(fifo_.begin(), fifo_.begin() + static_cast<std::ptrdiff_t>(column_bytes), chirp_bytes_.begin());
    fifo_.erase(fifo_.begin(), fifo_.begin() + static_cast<std::ptrdiff_t>(column_bytes));

    const auto nr = static_cast<Eigen::Index>(imaging_config_.nr);
    column.resize(nr);
    for (Eigen::Index row = 0; row < nr; ++row) {
        column(row) = sample_from_chirp(static_cast<std::size_t>(row));
    }
    ++columns_read_;
    return true;
}

Complex Dca1000ColumnSource::sample_from_chirp(std::size_t row) const {
    const std::size_t packed_rows = checked_packed_rows(reader_config_);
    const int lane = imaging_config_.channel_selection.channel_index;
    const auto* samples = reinterpret_cast<const int16_t*>(chirp_bytes_.data());
    const std::size_t sample_base = row * packed_rows;
    if (reader_config_.is_real) {
        return Complex(static_cast<double>(samples[sample_base + static_cast<std::size_t>(lane)]), 0.0);
    }
    return Complex(
        static_cast<double>(samples[sample_base + static_cast<std::size_t>(lane)]),
        static_cast<double>(samples[sample_base + static_cast<std::size_t>(reader_config_.num_lanes + lane)]));
}

EchoRingBuffer::EchoRingBuffer(std::size_t nr, std::size_t capacity_columns)
    : nr_(nr),
      capacity_columns_(capacity_columns),
      buffer_(static_cast<Eigen::Index>(nr), static_cast<Eigen::Index>(capacity_columns)) {
    if (nr_ == 0) {
        throw std::invalid_argument("EchoRingBuffer nr must be positive.");
    }
    if (capacity_columns_ == 0) {
        throw std::invalid_argument("EchoRingBuffer capacity must be positive.");
    }
}

void EchoRingBuffer::push_column(const VectorXc& column) {
    if (column.size() != static_cast<Eigen::Index>(nr_)) {
        throw std::invalid_argument("EchoRingBuffer column size does not match nr.");
    }
    buffer_.col(static_cast<Eigen::Index>(write_pos_)) = column;
    write_pos_ = (write_pos_ + 1) % capacity_columns_;
    size_ = std::min(size_ + 1, capacity_columns_);
    ++total_pushed_;
}

bool EchoRingBuffer::latest_window(std::size_t columns, MatrixXc& window) const {
    if (columns == 0) {
        throw std::invalid_argument("latest_window columns must be positive.");
    }
    if (columns > size_) {
        return false;
    }

    window.resize(static_cast<Eigen::Index>(nr_), static_cast<Eigen::Index>(columns));
    const std::size_t start = (write_pos_ + capacity_columns_ - columns) % capacity_columns_;
    for (std::size_t col = 0; col < columns; ++col) {
        const std::size_t source_col = (start + col) % capacity_columns_;
        window.col(static_cast<Eigen::Index>(col)) = buffer_.col(static_cast<Eigen::Index>(source_col));
    }
    return true;
}

std::size_t EchoRingBuffer::size() const {
    return size_;
}

std::size_t EchoRingBuffer::capacity() const {
    return capacity_columns_;
}

std::size_t EchoRingBuffer::total_pushed() const {
    return total_pushed_;
}

}  // namespace fft_imaging
