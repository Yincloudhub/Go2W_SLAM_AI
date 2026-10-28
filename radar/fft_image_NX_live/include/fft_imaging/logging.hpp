#pragma once

#include <cstddef>
#include <string>

namespace fft_imaging {

enum class LogLevel {
    Debug = 0,
    Info = 1,
    Warning = 2,
    Error = 3,
};

struct LogMessage {
    LogLevel level = LogLevel::Info;
    std::string component;
    std::string text;
};

class Logger {
public:
    virtual ~Logger() = default;
    virtual void log(const LogMessage& message) = 0;
};

class NullLogger final : public Logger {
public:
    void log(const LogMessage& message) override;
};

const char* log_level_name(LogLevel level);

}  // namespace fft_imaging
