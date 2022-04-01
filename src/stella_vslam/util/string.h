#ifndef STELLA_VSLAM_UTIL_STRING_H
#define STELLA_VSLAM_UTIL_STRING_H

#include <vector>
#include <string>
#include <sstream>

namespace stella_vslam {
namespace util {

inline std::vector<std::string> split_string(const std::string& str, const char del) {
    std::vector<std::string> splitted_strs;
    std::stringstream ss(str);
    std::string item;
    while (std::getline(ss, item, del)) {
        if (!item.empty()) {
            splitted_strs.push_back(item);
        }
    }
    return splitted_strs;
}

inline bool string_startswith(const std::string& str, const std::string& qry) {
    return str.size() >= qry.size()
           && std::equal(std::begin(qry), std::end(qry), std::begin(str));
}

} // namespace util
} // namespace stella_vslam

#endif // STELLA_VSLAM_UTIL_STRING_H
