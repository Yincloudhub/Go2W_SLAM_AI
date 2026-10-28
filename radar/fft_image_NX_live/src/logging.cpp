#include "fft_imaging/logging.hpp"

namespace fft_imaging {

void NullLogger::log(const LogMessage&) {}

const char* log_level_name(LogLevel level) {
    switch (level) {
        case LogLevel::Debug:
            return "DEBUG";
        case LogLevel::Info:
            return "INFO";
        case LogLevel::Warning:
            return "WARN";
        case LogLevel::Error:
            return "ERROR";
    }
    return "UNKNOWN";
}

}  // namespace fft_imaging
