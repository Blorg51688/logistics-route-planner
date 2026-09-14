#pragma once

#include <string>
#include <vector>

namespace logistics {

inline std::string trim(const std::string& s) {
    std::size_t begin = 0;
    std::size_t end = s.size();
    while (begin < end && (s[begin] == ' ' || s[begin] == '\t' || s[begin] == '\r' || s[begin] == '\n')) {
        ++begin;
    }
    while (end > begin && (s[end - 1] == ' ' || s[end - 1] == '\t' || s[end - 1] == '\r' || s[end - 1] == '\n')) {
        --end;
    }
    return s.substr(begin, end - begin);
}

inline std::vector<std::string> splitAndTrim(const std::string& s, char delim) {
    std::vector<std::string> parts;
    std::size_t start = 0;
    while (true) {
        const std::size_t pos = s.find(delim, start);
        if (pos == std::string::npos) {
            parts.push_back(trim(s.substr(start)));
            break;
        }
        parts.push_back(trim(s.substr(start, pos - start)));
        start = pos + 1;
    }
    return parts;
}

} // namespace logistics
