#pragma once

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace go2w {

struct HttpUrl {
    std::string host;
    int port = 80;
    std::string path = "/v1/chat/completions";
};

struct LlmHttpClientConfig {
    std::string url;
    std::string model = "local";
    int timeout_s = 20;
    int max_tokens = 256;
    double temperature = 0.0;
};

struct LlmHttpResult {
    bool ok = false;
    int status_code = 0;
    std::string content;
    std::string raw_body;
    std::string error;
};

HttpUrl parseHttpUrl(const std::string& url);
std::string extractOpenAiChatContent(const std::string& body);

class LlmHttpClient {
public:
    explicit LlmHttpClient(LlmHttpClientConfig config);

    LlmHttpResult chat(const std::vector<nlohmann::json>& messages) const;

private:
    LlmHttpClientConfig config_;
};

}  // namespace go2w
