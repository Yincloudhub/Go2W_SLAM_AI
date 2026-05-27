#include "go2w/llm_http_client.hpp"

#include <cerrno>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <netdb.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

namespace go2w {
namespace {

std::string trimAscii(std::string value)
{
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

std::string reasonFromErrno(const std::string& prefix)
{
    return prefix + ": " + std::strerror(errno);
}

void sendAll(int fd, const std::string& payload)
{
    const char* data = payload.data();
    std::size_t left = payload.size();
    while (left > 0) {
        const ssize_t sent = ::send(fd, data, left, 0);
        if (sent <= 0) throw std::runtime_error(reasonFromErrno("send failed"));
        data += sent;
        left -= static_cast<std::size_t>(sent);
    }
}

std::string receiveAll(int fd)
{
    std::string response;
    char buffer[4096];
    while (true) {
        const ssize_t n = ::recv(fd, buffer, sizeof(buffer), 0);
        if (n == 0) break;
        if (n < 0) throw std::runtime_error(reasonFromErrno("recv failed"));
        response.append(buffer, buffer + n);
    }
    return response;
}

int parseHttpStatus(const std::string& response)
{
    const auto line_end = response.find("\r\n");
    const std::string status_line = response.substr(0, line_end == std::string::npos ? response.size() : line_end);
    std::istringstream ss(status_line);
    std::string http;
    int status = 0;
    ss >> http >> status;
    return status;
}

std::string httpBody(const std::string& response)
{
    const auto split = response.find("\r\n\r\n");
    if (split == std::string::npos) return "";
    return response.substr(split + 4);
}

std::string httpPostJson(const HttpUrl& url, const std::string& body, int timeout_s, int* status_code)
{
    addrinfo hints {};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    addrinfo* result = nullptr;
    const std::string port = std::to_string(url.port);
    const int gai = ::getaddrinfo(url.host.c_str(), port.c_str(), &hints, &result);
    if (gai != 0) throw std::runtime_error(std::string("getaddrinfo failed: ") + gai_strerror(gai));

    int fd = -1;
    try {
        for (addrinfo* item = result; item != nullptr; item = item->ai_next) {
            fd = ::socket(item->ai_family, item->ai_socktype, item->ai_protocol);
            if (fd < 0) continue;
            timeval tv {};
            tv.tv_sec = timeout_s > 0 ? timeout_s : 20;
            tv.tv_usec = 0;
            ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
            ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
            if (::connect(fd, item->ai_addr, item->ai_addrlen) == 0) break;
            ::close(fd);
            fd = -1;
        }
        if (fd < 0) throw std::runtime_error(reasonFromErrno("connect failed"));

        std::ostringstream request;
        request << "POST " << url.path << " HTTP/1.1\r\n"
                << "Host: " << url.host << "\r\n"
                << "Content-Type: application/json\r\n"
                << "Accept: application/json\r\n"
                << "Connection: close\r\n"
                << "Content-Length: " << body.size() << "\r\n\r\n"
                << body;
        sendAll(fd, request.str());
        const std::string response = receiveAll(fd);
        if (status_code) *status_code = parseHttpStatus(response);
        ::close(fd);
        fd = -1;
        ::freeaddrinfo(result);
        return httpBody(response);
    } catch (...) {
        if (fd >= 0) ::close(fd);
        ::freeaddrinfo(result);
        throw;
    }
}

}  // namespace

HttpUrl parseHttpUrl(const std::string& raw_url)
{
    const std::string url = trimAscii(raw_url);
    const std::string prefix = "http://";
    if (url.rfind(prefix, 0) != 0) {
        throw std::invalid_argument("only http:// URLs are supported for local LLM service");
    }
    std::string rest = url.substr(prefix.size());
    const auto slash = rest.find('/');
    std::string host_port = slash == std::string::npos ? rest : rest.substr(0, slash);
    std::string path = slash == std::string::npos ? "/v1/chat/completions" : rest.substr(slash);
    if (path.empty()) path = "/v1/chat/completions";
    if (host_port.empty()) throw std::invalid_argument("missing host in LLM URL");

    HttpUrl parsed;
    parsed.path = path;
    const auto colon = host_port.rfind(':');
    if (colon != std::string::npos) {
        parsed.host = host_port.substr(0, colon);
        parsed.port = std::stoi(host_port.substr(colon + 1));
    } else {
        parsed.host = host_port;
        parsed.port = 80;
    }
    if (parsed.host.empty() || parsed.port <= 0) throw std::invalid_argument("invalid LLM URL");
    return parsed;
}

std::string extractOpenAiChatContent(const std::string& body)
{
    const auto json = nlohmann::json::parse(body);
    if (json.contains("choices") && json.at("choices").is_array() && !json.at("choices").empty()) {
        const auto& choice = json.at("choices").front();
        if (choice.contains("message") && choice.at("message").is_object() &&
            choice.at("message").contains("content") && choice.at("message").at("content").is_string()) {
            return choice.at("message").at("content").get<std::string>();
        }
        if (choice.contains("text") && choice.at("text").is_string()) return choice.at("text").get<std::string>();
    }
    if (json.contains("content") && json.at("content").is_string()) return json.at("content").get<std::string>();
    if (json.contains("response") && json.at("response").is_string()) return json.at("response").get<std::string>();
    return json.dump();
}

LlmHttpClient::LlmHttpClient(LlmHttpClientConfig config)
    : config_(std::move(config))
{
}

LlmHttpResult LlmHttpClient::chat(const std::vector<nlohmann::json>& messages) const
{
    LlmHttpResult result;
    try {
        const HttpUrl url = parseHttpUrl(config_.url);
        const nlohmann::json request = {
            {"model", config_.model},
            {"messages", messages},
            {"temperature", config_.temperature},
            {"max_tokens", config_.max_tokens},
            {"stream", false},
        };
        result.raw_body = httpPostJson(url, request.dump(), config_.timeout_s, &result.status_code);
        if (result.status_code < 200 || result.status_code >= 300) {
            result.ok = false;
            result.error = "http status " + std::to_string(result.status_code);
            return result;
        }
        result.content = extractOpenAiChatContent(result.raw_body);
        result.ok = true;
        return result;
    } catch (const std::exception& exc) {
        result.ok = false;
        result.error = exc.what();
        return result;
    }
}

}  // namespace go2w
