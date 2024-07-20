/**
 * @brief utils
 * @author postrantor
 * @date 2024
 * @copyright MIT License
 */

#include <vector>
#include <numeric>
#include <string>
#include <sstream>

template <typename T>
std::string array_to_string(
    const std::vector<T>& array, const std::string& prefix, const std::string& separator = ", ") {
  if (array.empty()) {
    return prefix + "[]";
  }

  std::string result = std::accumulate(
      std::next(array.begin()), array.end(), std::to_string(array[0]),
      [&separator](const std::string& a, T b) { return a + separator + std::to_string(b); });

  return prefix + "[" + result + "]";
}
