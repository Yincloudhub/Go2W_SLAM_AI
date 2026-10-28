#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include "fft_imaging/config.hpp"
#include "fft_imaging/types.hpp"

namespace fft_imaging {

class ComplexColumnSource {
public:
    virtual ~ComplexColumnSource() = default;
    virtual bool next_column(VectorXc& column) = 0;
    virtual std::size_t columns_read() const = 0;
};

class BinReplaySource final : public ComplexColumnSource {
public:
    BinReplaySource(
        std::filesystem::path bin_path,
        ReaderConfig reader_config,
        ImagingConfig imaging_config);

    bool next_column(VectorXc& column) override;
    std::size_t columns_read() const override;

    std::size_t bytes_per_column() const;

private:
    std::filesystem::path bin_path_;
    ReaderConfig reader_config_;
    ImagingConfig imaging_config_;
    std::ifstream input_;
    std::size_t columns_read_ = 0;
    std::vector<int16_t> sample_buffer_;
};

class Dca1000ColumnSource : public ComplexColumnSource {
public:
    Dca1000ColumnSource(
        std::string bind_ip,
        int data_port,
        ReaderConfig reader_config,
        ImagingConfig imaging_config,
        int timeout_ms = 2000,
        int socket_receive_buffer_bytes = 268435456);

    ~Dca1000ColumnSource() override;

    bool next_column(VectorXc& column) override;
    std::size_t columns_read() const override;

    std::size_t packets_received() const;
    std::size_t sequence_gaps() const;
    std::size_t bytes_per_column() const;

private:
    bool recv_payload();
    void append_payload_bytes(const uint8_t* data, std::size_t size);
    bool pop_column(VectorXc& column);
    Complex sample_from_chirp(std::size_t row) const;

    std::string bind_ip_;
    int data_port_ = 4098;
    ReaderConfig reader_config_;
    ImagingConfig imaging_config_;
    int timeout_ms_ = 2000;
    int socket_receive_buffer_bytes_ = 268435456;
    int socket_fd_ = -1;
    bool aligned_ = false;
    std::uint64_t expected_next_data_start_ = 0;
    std::size_t packets_received_ = 0;
    std::size_t sequence_gaps_ = 0;
    std::vector<uint8_t> fifo_;
    std::vector<uint8_t> chirp_bytes_;
    std::vector<uint8_t> packet_buffer_;
    std::size_t columns_read_ = 0;
};

class EchoRingBuffer {
public:
    EchoRingBuffer(std::size_t nr, std::size_t capacity_columns);

    void push_column(const VectorXc& column);
    bool latest_window(std::size_t columns, MatrixXc& window) const;
    std::size_t size() const;
    std::size_t capacity() const;
    std::size_t total_pushed() const;

private:
    std::size_t nr_ = 0;
    std::size_t capacity_columns_ = 0;
    std::size_t write_pos_ = 0;
    std::size_t size_ = 0;
    std::size_t total_pushed_ = 0;
    MatrixXc buffer_;
};

}  // namespace fft_imaging
