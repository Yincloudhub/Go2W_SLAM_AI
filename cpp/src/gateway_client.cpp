#include "go2w/gateway_client.hpp"

#include <chrono>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <utility>

#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

namespace go2w {
namespace {

void closeFd(int& fd)
{
    if (fd >= 0) {
        close(fd);
        fd = -1;
    }
}

void setNonBlocking(int fd)
{
    const int flags = fcntl(fd, F_GETFL, 0);
    if (flags >= 0) fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

void writeAll(int fd, const std::string& data)
{
    const char* cursor = data.data();
    std::size_t left = data.size();
    while (left > 0) {
        const ssize_t written = write(fd, cursor, left);
        if (written < 0) {
            if (errno == EINTR) continue;
            throw std::runtime_error(std::string("write failed: ") + std::strerror(errno));
        }
        if (written == 0) throw std::runtime_error("write returned zero");
        cursor += written;
        left -= static_cast<std::size_t>(written);
    }
}

void readAvailable(int fd, std::string& out, bool& open)
{
    char buffer[4096];
    while (true) {
        const ssize_t n = read(fd, buffer, sizeof(buffer));
        if (n > 0) {
            out.append(buffer, static_cast<std::size_t>(n));
            continue;
        }
        if (n == 0) {
            open = false;
            return;
        }
        if (errno == EINTR) continue;
        if (errno == EAGAIN || errno == EWOULDBLOCK) return;
        open = false;
        return;
    }
}

std::vector<nlohmann::json> extractJsonObjectsLocal(const std::string& text)
{
    std::vector<nlohmann::json> objects;
    for (std::size_t start = 0; start < text.size(); ++start) {
        if (text[start] != '{') continue;
        bool in_string = false;
        bool escaped = false;
        int depth = 0;
        for (std::size_t i = start; i < text.size(); ++i) {
            const char ch = text[i];
            if (in_string) {
                if (escaped) {
                    escaped = false;
                } else if (ch == '\\') {
                    escaped = true;
                } else if (ch == '"') {
                    in_string = false;
                }
                continue;
            }
            if (ch == '"') {
                in_string = true;
            } else if (ch == '{') {
                ++depth;
            } else if (ch == '}') {
                --depth;
                if (depth == 0) {
                    try {
                        objects.push_back(nlohmann::json::parse(text.substr(start, i - start + 1)));
                    } catch (...) {
                    }
                    start = i;
                    break;
                }
            }
        }
    }
    return objects;
}

}  // namespace

ProcessResult runProcessWithInput(const std::vector<std::string>& argv, const std::string& input, int timeout_s)
{
    if (argv.empty()) throw std::runtime_error("empty argv");

    int stdin_pipe[2] {-1, -1};
    int stdout_pipe[2] {-1, -1};
    int stderr_pipe[2] {-1, -1};
    if (pipe(stdin_pipe) != 0 || pipe(stdout_pipe) != 0 || pipe(stderr_pipe) != 0) {
        throw std::runtime_error(std::string("pipe failed: ") + std::strerror(errno));
    }

    const pid_t pid = fork();
    if (pid < 0) {
        throw std::runtime_error(std::string("fork failed: ") + std::strerror(errno));
    }

    if (pid == 0) {
        dup2(stdin_pipe[0], STDIN_FILENO);
        dup2(stdout_pipe[1], STDOUT_FILENO);
        dup2(stderr_pipe[1], STDERR_FILENO);
        close(stdin_pipe[0]);
        close(stdin_pipe[1]);
        close(stdout_pipe[0]);
        close(stdout_pipe[1]);
        close(stderr_pipe[0]);
        close(stderr_pipe[1]);

        std::vector<char*> c_argv;
        c_argv.reserve(argv.size() + 1);
        for (const auto& item : argv) c_argv.push_back(const_cast<char*>(item.c_str()));
        c_argv.push_back(nullptr);
        execv(c_argv[0], c_argv.data());
        _exit(127);
    }

    closeFd(stdin_pipe[0]);
    closeFd(stdout_pipe[1]);
    closeFd(stderr_pipe[1]);

    ProcessResult result;
    try {
        writeAll(stdin_pipe[1], input);
    } catch (...) {
        closeFd(stdin_pipe[1]);
        kill(pid, SIGTERM);
        throw;
    }
    closeFd(stdin_pipe[1]);

    setNonBlocking(stdout_pipe[0]);
    setNonBlocking(stderr_pipe[0]);
    bool stdout_open = true;
    bool stderr_open = true;
    bool exited = false;
    bool killed = false;
    int status = 0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(timeout_s);

    while (stdout_open || stderr_open || !exited) {
        if (!exited) {
            const pid_t wait_result = waitpid(pid, &status, WNOHANG);
            if (wait_result == pid) {
                exited = true;
            }
        }

        if (!exited && std::chrono::steady_clock::now() > deadline) {
            result.timed_out = true;
            if (!killed) {
                kill(pid, SIGTERM);
                killed = true;
            } else {
                kill(pid, SIGKILL);
            }
        }

        pollfd fds[2];
        int nfds = 0;
        if (stdout_open) fds[nfds++] = {stdout_pipe[0], POLLIN | POLLHUP | POLLERR, 0};
        if (stderr_open) fds[nfds++] = {stderr_pipe[0], POLLIN | POLLHUP | POLLERR, 0};
        if (nfds > 0) {
            poll(fds, nfds, 100);
            int index = 0;
            if (stdout_open) {
                readAvailable(stdout_pipe[0], result.stdout_text, stdout_open);
                ++index;
            }
            if (stderr_open) {
                readAvailable(stderr_pipe[0], result.stderr_text, stderr_open);
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    closeFd(stdout_pipe[0]);
    closeFd(stderr_pipe[0]);
    if (WIFEXITED(status)) {
        result.exit_code = WEXITSTATUS(status);
    } else if (WIFSIGNALED(status)) {
        result.exit_code = 128 + WTERMSIG(status);
    } else {
        result.exit_code = status;
    }
    return result;
}

GatewayClient::GatewayClient(GatewayClientConfig config)
    : config_(std::move(config))
{
}

GatewayClientResult GatewayClient::send(const nlohmann::json& command) const
{
    const std::string payload = command.dump() + "\n";
    ProcessResult process = runProcessWithInput({config_.client_path, config_.network_interface}, payload, config_.timeout_s);
    if (process.timed_out) {
        throw std::runtime_error("gateway client timed out");
    }
    if (process.exit_code != 0) {
        throw std::runtime_error("gateway client failed: " + process.stderr_text);
    }

    const auto objects = extractJsonObjectsLocal(process.stdout_text);
    for (const auto& object : objects) {
        if (object.contains("world_state")) return {object, process};
    }
    for (const auto& object : objects) {
        if (object.contains("accepted")) return {object, process};
    }
    if (!objects.empty()) return {objects.back(), process};
    throw std::runtime_error("gateway did not return JSON: " + process.stdout_text);
}

}  // namespace go2w
