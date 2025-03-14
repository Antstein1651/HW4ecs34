#include "StringUtils.h"
#include <algorithm>
#include <cctype>
#include <sstream>

namespace StringUtils {

std::string Slice(const std::string &str, ssize_t start, ssize_t end) noexcept {
    if (end == 0) end = str.size();
    if (start < 0) start += str.size();
    if (end < 0) end += str.size();
    if (start < 0) start = 0;
    if (end > static_cast<ssize_t>(str.size())) end = str.size();
    if (start >= end) return "";
    return str.substr(start, end - start);
}

std::string Capitalize(const std::string &str) noexcept {
    if (str.empty()) return str;
    std::string result = str;
    result[0] = std::toupper(result[0]);
    std::transform(result.begin() + 1, result.end(), result.begin() + 1, ::tolower);
    return result;
}

std::string Upper(const std::string &str) noexcept {
    std::string result = str;
    std::transform(result.begin(), result.end(), result.begin(), ::toupper);
    return result;
}

std::string Lower(const std::string &str) noexcept {
    std::string result = str;
    std::transform(result.begin(), result.end(), result.begin(), ::tolower);
    return result;
}

std::string LStrip(const std::string &str) noexcept {
    size_t start = str.find_first_not_of(" \t\n\r");
    return (start == std::string::npos) ? "" : str.substr(start);
}

std::string RStrip(const std::string &str) noexcept {
    size_t end = str.find_last_not_of(" \t\n\r");
    return (end == std::string::npos) ? "" : str.substr(0, end + 1);
}

std::string Strip(const std::string &str) noexcept {
    return LStrip(RStrip(str));
}

std::string Center(const std::string &str, int width, char fill) noexcept {
    if (static_cast<size_t>(width) <= str.size()) return str;
    size_t padding = width - str.size();
    size_t left = padding / 2;
    size_t right = padding - left;
    return std::string(left, fill) + str + std::string(right, fill);
}

std::string LJust(const std::string &str, int width, char fill) noexcept {
    if (static_cast<size_t>(width) <= str.size()) return str;
    return str + std::string(width - str.size(), fill);
}

std::string RJust(const std::string &str, int width, char fill) noexcept {
    if (static_cast<size_t>(width) <= str.size()) return str;
    return std::string(width - str.size(), fill) + str;
}

std::string Replace(const std::string &str, const std::string &old, const std::string &rep) noexcept {
    if (old.empty()) return str;
    std::string result = str;
    size_t pos = 0;
    while ((pos = result.find(old, pos)) != std::string::npos) {
        result.replace(pos, old.length(), rep);
        pos += rep.length();
    }
    return result;
}

std::vector<std::string> Split(const std::string &str, const std::string &splt) noexcept {
    std::vector<std::string> result;
    if (splt.empty()) {
        std::istringstream stream(str);
        std::string word;
        while (stream >> word) {
            result.push_back(word);
        }
    } else {
        size_t start = 0, end;
        while ((end = str.find(splt, start)) != std::string::npos) {
            result.push_back(str.substr(start, end - start));
            start = end + splt.length();
        }
        result.push_back(str.substr(start));
    }
    return result;
}

std::string Join(const std::string &str, const std::vector<std::string> &vect) noexcept {
    if (vect.empty()) return "";
    std::ostringstream joined;
    for (size_t i = 0; i < vect.size(); ++i) {
        if (i > 0) joined << str;
        joined << vect[i];
    }
    return joined.str();
}

std::string ExpandTabs(const std::string &str, int tabsize) noexcept {
    std::string result;
    for (char ch : str) {
        if (ch == '\t') {
            result.append(tabsize - result.size() % tabsize, ' ');
        } else {
            result += ch;
        }
    }
    return result;
}

int EditDistance(const std::string &left, const std::string &right, bool ignorecase) noexcept {
    std::string l = left, r = right;
    if (ignorecase) {
        std::transform(l.begin(), l.end(), l.begin(), ::tolower);
        std::transform(r.begin(), r.end(), r.begin(), ::tolower);
    }
    size_t len1 = l.size(), len2 = r.size();
    std::vector<std::vector<int>> dp(len1 + 1, std::vector<int>(len2 + 1, 0));
    for (size_t i = 0; i <= len1; ++i) dp[i][0] = i;
    for (size_t j = 0; j <= len2; ++j) dp[0][j] = j;
    for (size_t i = 1; i <= len1; ++i) {
        for (size_t j = 1; j <= len2; ++j) {
            if (l[i - 1] == r[j - 1]) {
                dp[i][j] = dp[i - 1][j - 1];
            } else {
                dp[i][j] = std::min({dp[i - 1][j] + 1, dp[i][j - 1] + 1, dp[i - 1][j - 1] + 1});
            }
        }
    }
    return dp[len1][len2];
}

}  // namespace StringUtils
