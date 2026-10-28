cmake_minimum_required(VERSION 3.16)
project(fft_imaging_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgcodecs imgproc videoio)
pkg_check_modules(FFTW3 REQUIRED IMPORTED_TARGET fftw3)
pkg_check_modules(MATIO REQUIRED IMPORTED_TARGET matio)

add_library(fft_imaging_core
    src/matrix_summary.cpp
    src/raw_capture_reader.cpp
    src/realtime_stream.cpp
    src/session.cpp
    src/matrix_ops.cpp
    src/fft_engine.cpp
    src/spline_interp1.cpp
    src/spline_interp2.cpp
    src/core_pipeline.cpp
    src/range_angle_imager.cpp
)

target_include_directories(fft_imaging_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(fft_imaging_core
    PUBLIC
        Eigen3::Eigen
        PkgConfig::FFTW3
)

target_compile_options(fft_imaging_core
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)

add_executable(fft_imaging_cli
    app/fft_imaging_cli.cpp
)

add_library(fft_imaging_runtime
    src/backend.cpp
    src/config.cpp
    src/logging.cpp
    src/profiler.cpp
    src/runtime.cpp
)
target_include_directories(fft_imaging_runtime
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(fft_imaging_runtime
    PUBLIC
        fft_imaging_core
)
target_compile_options(fft_imaging_runtime
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)

add_library(fft_imaging_tooling
    src/image_writer.cpp
    src/vfit_reader.cpp
    src/tool_adapters.cpp
)
target_include_directories(fft_imaging_tooling
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(fft_imaging_tooling
    PUBLIC
        fft_imaging_runtime
        PkgConfig::MATIO
        ${OpenCV_LIBS}
)
target_compile_options(fft_imaging_tooling
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)

target_link_libraries(fft_imaging_cli PRIVATE fft_imaging_tooling)

add_executable(fft_imaging_live
    app/fft_imaging_live.cpp
)

target_link_libraries(fft_imaging_live PRIVATE fft_imaging_tooling)
