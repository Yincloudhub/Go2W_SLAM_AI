#include "go2w/llm_http_client.hpp"

#include <iostream>
#include <stdexcept>

namespace {

void require(bool condition, const std::string& message)
{
    if (!condition) throw std::runtime_error(message);
}

}  // namespace

int main()
{
    const auto url = go2w::parseHttpUrl("http://127.0.0.1:8080/v1/chat/completions");
    require(url.host == "127.0.0.1", "host mismatch");
    require(url.port == 8080, "port mismatch");
    require(url.path == "/v1/chat/completions", "path mismatch");

    const std::string body = R"({"choices":[{"message":{"content":"{\"reply\":\"ok\",\"targets\":[\"wp_1\"]}"}}]})";
    const std::string content = go2w::extractOpenAiChatContent(body);
    require(content.find("\"targets\"") != std::string::npos, "content extraction failed");
    std::cout << content << std::endl;
    return 0;
}
