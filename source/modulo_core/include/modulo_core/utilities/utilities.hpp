#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>


namespace modulo::core::utilities {

/**
 * @brief Parse a string argument value from an argument list given a pattern prefix.
 * @param args a vector of string arguments
 * @param pattern the prefix pattern to find and strip
 * @param result the default argument value that is overwritten by reference if the given pattern is found
 * @return the value of the resultant string
 */
static std::string parse_string_argument(const std::vector<std::string>& args, const std::string& pattern, std::string& result) {
  for (const auto& arg : args) {
    std::string::size_type index = arg.find(pattern);
    if (index != std::string::npos) {
      result = arg;
      result.erase(index, pattern.length());
      break;
    }
  }
  return result;
}

/**
 * @brief Parse a string node name from NodeOptions arguments.
 * @param options the NodeOptions structure to parse
 * @param fallback the default name if the NodeOptions structure cannot be parsed
 * @return the parsed node name or the fallback name
 */
std::string parse_node_name(const rclcpp::NodeOptions& options, const std::string& fallback="") {
  std::string node_name(fallback);
  const std::string pattern("__node:=");
  return parse_string_argument(options.arguments(), pattern, node_name);
}

/**
 * @brief Parse a string node namespace from NodeOptions arguments.
 * @param options the NodeOptions structure to parse
 * @param fallback the default namespace if the NodeOptions structure cannot be parsed
 * @return the parsed node namespace or the fallback namespace
 */
std::string parse_node_namespace(const rclcpp::NodeOptions& options, const std::string& fallback="") {
  std::string node_namespace(fallback);
  const std::string pattern("__ns:=");
  return parse_string_argument(options.arguments(), pattern, node_namespace);
}

/**
 * @brief Parse a period in seconds from NodeOptions parameters.
 * @details The parameter must have the name "period"
 * and be interpretable as a value in seconds of type double
 * @param options the NodeOptions structure to parse
 * @param default_period the default period in seconds
 * @return the parsed period or the default period as a chrono duration
 */
std::chrono::nanoseconds parse_period(const rclcpp::NodeOptions& options, double default_period = 0.001) {
  double period = default_period;
  for (auto& param : options.parameter_overrides()) {
    if (param.get_name() == "period") {
      period = param.as_double();
      break;
    }
  }
  return std::chrono::nanoseconds(static_cast<int64_t>(period * 1e9));
}

}