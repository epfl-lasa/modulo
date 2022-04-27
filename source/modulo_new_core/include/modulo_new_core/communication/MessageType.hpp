#pragma once

namespace modulo_new_core::communication {

/**
 * @brief Enum of all supported ROS message types for the MessagePairInterface
 * @see MessagePairInterface
 */
enum class MessageType {
  BOOL, FLOAT64, FLOAT64_MULTI_ARRAY, INT32, STRING, ENCODED_STATE
};

}// namespace modulo_new_core::communication