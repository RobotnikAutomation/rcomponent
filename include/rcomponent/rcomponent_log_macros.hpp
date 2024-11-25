#ifndef _RCOMPONENT_LOG_
#define _RCOMPONENT_LOG_

// debug
#define RCOMPONENT_DEBUG(logger, format_string, ...)                                                                           \
  RCLCPP_DEBUG(logger, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__, ##__VA_ARGS__)

#define RCOMPONENT_DEBUG_NAMED(logger, name, format_string, ...)                                                               \
  RCLCPP_DEBUG_NAMED(logger, name, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__,            \
                  ##__VA_ARGS__)

#define RCOMPONENT_DEBUG_COND(logger, cond, format_string, ...)                                                                \
  RCLCPP_DEBUG_COND(logger, cond, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__,             \
                 ##__VA_ARGS__)

#define RCOMPONENT_DEBUG_COND_NAMED(logger, cond, name, format_string, ...)                                                    \
  RCLCPP_DEBUG_COND_NAMED(logger, cond, name, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__, \
                       ##__VA_ARGS__)

#define RCOMPONENT_DEBUG_ONCE(logger, format_string, ...)                                                                      \
  RCLCPP_DEBUG_ONCE(logger, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__, ##__VA_ARGS__)

#define RCOMPONENT_DEBUG_ONCE_NAMED(logger, name, format_string, ...)                                                          \
  RCLCPP_DEBUG_ONCE_NAMED(logger, name, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__,       \
                       ##__VA_ARGS__)

#define RCOMPONENT_DEBUG_THROTTLE(logger, rate, format_string, ...)                                                            \
  RCLCPP_DEBUG_THROTTLE(logger, rate, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__,         \
                     ##__VA_ARGS__)

#define RCOMPONENT_DEBUG_THROTTLE_NAMED(logger, rate, name, format_string, ...)                                                \
  RCLCPP_DEBUG_THROTTLE_NAMED(logger, rate, name, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__,       \
                           __LINE__, ##__VA_ARGS__)

#define RCOMPONENT_DEBUG_DELAYED_THROTTLE(logger, rate, format_string, ...)                                                    \
  RCLCPP_DEBUG_DELAYED_THROTTLE(logger, rate, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__, \
                             ##__VA_ARGS__)

#define RCOMPONENT_DEBUG_DELAYED_THROTTLE_NAMED(logger, rate, name, format_string, ...)                                        \
  RCLCPP_DEBUG_DELAYED_THROTTLE_NAMED(logger, rate, name, "%s::%s::%d: " format_string, this->component_name.c_str(),             \
                                   __FUNCTION__, __LINE__, ##__VA_ARGS__)

#define RCOMPONENT_DEBUG_FILTER(logger, filter, format_string, ...)                                                            \
  RCLCPP_DEBUG_FILTER(logger, filter, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__,         \
                   ##__VA_ARGS__)

#define RCOMPONENT_DEBUG_FILTER_NAMED(logger, filter, name, format_string, ...)                                                \
  RCLCPP_DEBUG_FILTER_NAMED(logger, filter, name, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__,       \
                         __LINE__, ##__VA_ARGS__)

#define RCOMPONENT_DEBUG_STREAM(logger, args)                                                                                  \
  RCLCPP_DEBUG_STREAM(logger, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " << args)

#define RCOMPONENT_DEBUG_STREAM_NAMED(logger, name, args)                                                                      \
  RCLCPP_DEBUG_STREAM_NAMED(logger, name, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " << args)

#define RCOMPONENT_DEBUG_STREAM_COND(logger, cond, args)                                                                       \
  RCLCPP_DEBUG_STREAM_COND(logger, cond, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " << args)

#define RCOMPONENT_DEBUG_STREAM_COND_NAMED(logger, cond, name, args)                                                           \
  RCLCPP_DEBUG_STREAM_COND_NAMED(logger, cond, name, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": "     \
                                                               << args)

#define RCOMPONENT_DEBUG_STREAM_ONCE(logger, args)                                                                             \
  RCLCPP_DEBUG_STREAM_ONCE(logger, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " << args)

#define RCOMPONENT_DEBUG_STREAM_ONCE_NAMED(logger, name, args)                                                                 \
  RCLCPP_DEBUG_STREAM_ONCE_NAMED(logger, name, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " << args)

#define RCOMPONENT_DEBUG_STREAM_THROTTLE(logger, rate, args)                                                                   \
  RCLCPP_DEBUG_STREAM_THROTTLE(logger, rate, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " << args)

#define RCOMPONENT_DEBUG_STREAM_THROTTLE_NAMED(logger, rate, name, args)                                                       \
  RCLCPP_DEBUG_STREAM_THROTTLE_NAMED(logger, rate, name, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " \
                                                                   << args)
#define RCOMPONENT_DEBUG_STREAM_DELAYED_THROTTLE(logger, rate, args)                                                           \
  define RCLCPP_DEBUG_STREAM_DELAYED_THROTTLE(logger, rate, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__      \
                                                                      << ": " << args)
#define RCOMPONENT_DEBUG_STREAM_DELAYED_THROTTLE_NAMED(logger, rate, name, args)                                               \
  RCLCPP_DEBUG_STREAM_DELAYED_THROTTLE_NAMED(logger, rate, name, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ \
                                                                           << ": " << args)
#define RCOMPONENT_DEBUG_STREAM_FILTER(logger, filter, args)                                                                   \
  RCLCPP_DEBUG_STREAM_FILTER(logger, filter, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " << args)

#define RCOMPONENT_DEBUG_STREAM_FILTER_NAMED(logger, filter, name, args)                                                       \
  RCLCPP_DEBUG_STREAM_FILTER_NAMED(logger, filter, name, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " \
                                                                   << args)

// info
#define RCOMPONENT_INFO(logger, format_string, ...)                                                                            \
  RCLCPP_INFO(logger, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__, ##__VA_ARGS__)

#define RCOMPONENT_INFO_NAMED(logger, name, format_string, ...)                                                                \
  RCLCPP_INFO_NAMED(logger, name, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__,             \
                 ##__VA_ARGS__)

#define RCOMPONENT_INFO_COND(logger, cond, format_string, ...)                                                                 \
  RCLCPP_INFO_COND(logger, cond, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__, ##__VA_ARGS__)

#define RCOMPONENT_INFO_COND_NAMED(logger, cond, name, format_string, ...)                                                     \
  RCLCPP_INFO_COND_NAMED(logger, cond, name, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__,  \
                      ##__VA_ARGS__)

#define RCOMPONENT_INFO_ONCE(logger, format_string, ...)                                                                       \
  RCLCPP_INFO_ONCE(logger, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__, ##__VA_ARGS__)

#define RCOMPONENT_INFO_ONCE_NAMED(logger, name, format_string, ...)                                                           \
  RCLCPP_INFO_ONCE_NAMED(logger, name, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__,        \
                      ##__VA_ARGS__)

#define RCOMPONENT_INFO_THROTTLE(logger, rate, format_string, ...)                                                             \
  RCLCPP_INFO_THROTTLE(logger, rate, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__,          \
                    ##__VA_ARGS__)

#define RCOMPONENT_INFO_THROTTLE_NAMED(logger, rate, name, format_string, ...)                                                 \
  RCLCPP_INFO_THROTTLE_NAMED(logger, rate, name, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__,        \
                          __LINE__, ##__VA_ARGS__)

#define RCOMPONENT_INFO_DELAYED_THROTTLE(logger, rate, format_string, ...)                                                     \
  RCLCPP_INFO_DELAYED_THROTTLE(logger, rate, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__,  \
                            ##__VA_ARGS__)

#define RCOMPONENT_INFO_DELAYED_THROTTLE_NAMED(logger, rate, name, format_string, ...)                                         \
  RCLCPP_INFO_DELAYED_THROTTLE_NAMED(logger, rate, name, "%s::%s::%d: " format_string, this->component_name.c_str(),              \
                                  __FUNCTION__, __LINE__, ##__VA_ARGS__)

#define RCOMPONENT_INFO_FILTER(logger, filter, format_string, ...)                                                             \
  RCLCPP_INFO_FILTER(logger, filter, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__,          \
                  ##__VA_ARGS__)

#define RCOMPONENT_INFO_FILTER_NAMED(logger, filter, name, format_string, ...)                                                 \
  RCLCPP_INFO_FILTER_NAMED(logger, filter, name, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__,        \
                        __LINE__, ##__VA_ARGS__)

#define RCOMPONENT_INFO_STREAM(logger, args)                                                                                   \
  RCLCPP_INFO_STREAM(logger, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " << args)

#define RCOMPONENT_INFO_STREAM_NAMED(logger, name, args)                                                                       \
  RCLCPP_INFO_STREAM_NAMED(logger, name, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " << args)

#define RCOMPONENT_INFO_STREAM_COND(logger, cond, args)                                                                        \
  RCLCPP_INFO_STREAM_COND(logger, cond, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " << args)

#define RCOMPONENT_INFO_STREAM_COND_NAMED(logger, cond, name, args)                                                            \
  RCLCPP_INFO_STREAM_COND_NAMED(logger, cond, name, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": "      \
                                                              << args)

#define RCOMPONENT_INFO_STREAM_ONCE(logger, args)                                                                              \
  RCLCPP_INFO_STREAM_ONCE(logger, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " << args)

#define RCOMPONENT_INFO_STREAM_ONCE_NAMED                                                                              \
  (logger, name, args)                                                                                                         \
      RCLCPP_INFO_STREAM_ONCE(logger, name, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " << args)

#define RCOMPONENT_INFO_STREAM_THROTTLE(logger, rate, args)                                                                    \
  RCLCPP_INFO_STREAM_THROTTLE(logger, rate, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " << args)

#define RCOMPONENT_INFO_STREAM_THROTTLE_NAMED(logger, rate, name, args)                                                        \
  RCLCPP_INFO_STREAM_THROTTLE_NAMED(logger, rate, name, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": "  \
                                                                  << args)
#define RCOMPONENT_INFO_STREAM_DELAYED_THROTTLE(logger, rate, args)                                                            \
  define RCLCPP_INFO_STREAM_DELAYED_THROTTLE(logger, rate, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__       \
                                                                     << ": " << args)
#define RCOMPONENT_INFO_STREAM_DELAYED_THROTTLE_NAMED(logger, rate, name, args)                                                \
  RCLCPP_INFO_STREAM_DELAYED_THROTTLE_NAMED(logger, rate, name, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__  \
                                                                          << ": " << args)
#define RCOMPONENT_INFO_STREAM_FILTER(logger, filter, args)                                                                    \
  RCLCPP_INFO_STREAM_FILTER(logger, filter, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " << args)

#define RCOMPONENT_INFO_STREAM_FILTER_NAMED(logger, filter, name, args)                                                        \
  RCLCPP_INFO_STREAM_FILTER_NAMED(logger, filter, name, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": "  \
                                                                  << args)

// warn
#define RCOMPONENT_WARN(logger, format_string, ...)                                                                            \
  RCLCPP_WARN(logger, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__, ##__VA_ARGS__)

#define RCOMPONENT_WARN_NAMED(logger, name, format_string, ...)                                                                \
  RCLCPP_WARN_NAMED(logger, name, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__,             \
                 ##__VA_ARGS__)

#define RCOMPONENT_WARN_COND(logger, cond, format_string, ...)                                                                 \
  RCLCPP_WARN_COND(logger, cond, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__, ##__VA_ARGS__)

#define RCOMPONENT_WARN_COND_NAMED(logger, cond, name, format_string, ...)                                                     \
  RCLCPP_WARN_COND_NAMED(logger, cond, name, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__,  \
                      ##__VA_ARGS__)

#define RCOMPONENT_WARN_ONCE(logger, format_string, ...)                                                                       \
  RCLCPP_WARN_ONCE(logger, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__, ##__VA_ARGS__)

#define RCOMPONENT_WARN_ONCE_NAMED(logger, name, format_string, ...)                                                           \
  RCLCPP_WARN_ONCE_NAMED(logger, name, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__,        \
                      ##__VA_ARGS__)

#define RCOMPONENT_WARN_THROTTLE(logger, rate, format_string, ...)                                                             \
  RCLCPP_WARN_THROTTLE(logger, rate, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__,          \
                    ##__VA_ARGS__)

#define RCOMPONENT_WARN_THROTTLE_NAMED(logger, rate, name, format_string, ...)                                                 \
  RCLCPP_WARN_THROTTLE_NAMED(logger, rate, name, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__,        \
                          __LINE__, ##__VA_ARGS__)

#define RCOMPONENT_WARN_DELAYED_THROTTLE(logger, rate, format_string, ...)                                                     \
  RCLCPP_WARN_DELAYED_THROTTLE(logger, rate, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__,  \
                            ##__VA_ARGS__)

#define RCOMPONENT_WARN_DELAYED_THROTTLE_NAMED(logger, rate, name, format_string, ...)                                         \
  RCLCPP_WARN_DELAYED_THROTTLE_NAMED(logger, rate, name, "%s::%s::%d: " format_string, this->component_name.c_str(),              \
                                  __FUNCTION__, __LINE__, ##__VA_ARGS__)

#define RCOMPONENT_WARN_FILTER(logger, filter, format_string, ...)                                                             \
  RCLCPP_WARN_FILTER(logger, filter, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__,          \
                  ##__VA_ARGS__)

#define RCOMPONENT_WARN_FILTER_NAMED(logger, filter, name, format_string, ...)                                                 \
  RCLCPP_WARN_FILTER_NAMED(logger, filter, name, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__,        \
                        __LINE__, ##__VA_ARGS__)

#define RCOMPONENT_WARN_STREAM(logger, args)                                                                                   \
  RCLCPP_WARN_STREAM(logger, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " << args)

#define RCOMPONENT_WARN_STREAM_NAMED(logger, name, args)                                                                       \
  RCLCPP_WARN_STREAM_NAMED(logger, name, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " << args)

#define RCOMPONENT_WARN_STREAM_COND(logger, cond, args)                                                                        \
  RCLCPP_WARN_STREAM_COND(logger, cond, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " << args)

#define RCOMPONENT_WARN_STREAM_COND_NAMED(logger, cond, name, args)                                                            \
  RCLCPP_WARN_STREAM_COND_NAMED(logger, cond, name, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": "      \
                                                              << args)

#define RCOMPONENT_WARN_STREAM_ONCE(logger, args)                                                                              \
  RCLCPP_WARN_STREAM_ONCE(logger, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " << args)

#define RCOMPONENT_WARN_STREAM_ONCE_NAMED                                                                              \
  (logger, name, args)                                                                                                         \
      RCLCPP_WARN_STREAM_ONCE(logger, name, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " << args)

#define RCOMPONENT_WARN_STREAM_THROTTLE(logger, clock, rate, args)                                                                    \
  RCLCPP_WARN_STREAM_THROTTLE(logger, clock, rate, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " << args)

#define RCOMPONENT_WARN_STREAM_THROTTLE_NAMED(logger, clock, rate, name, args)                                                        \
  RCLCPP_WARN_STREAM_THROTTLE_NAMED(logger, clock, rate, name, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": "  \
                                                                  << args)
#define RCOMPONENT_WARN_STREAM_DELAYED_THROTTLE(logger, clock, rate, args)                                                            \
  define RCLCPP_WARN_STREAM_DELAYED_THROTTLE(logger, clock, rate, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__       \
                                                                     << ": " << args)
#define RCOMPONENT_WARN_STREAM_DELAYED_THROTTLE_NAMED(logger, rate, name, args)                                                \
  RCLCPP_WARN_STREAM_DELAYED_THROTTLE_NAMED(logger, rate, name, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__  \
                                                                          << ": " << args)
#define RCOMPONENT_WARN_STREAM_FILTER(logger, filter, args)                                                                    \
  RCLCPP_WARN_STREAM_FILTER(logger, filter, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " << args)

#define RCOMPONENT_WARN_STREAM_FILTER_NAMED(logger, filter, name, args)                                                        \
  RCLCPP_WARN_STREAM_FILTER_NAMED(logger, filter, name, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": "  \
                                                                  << args)

// error
#define RCOMPONENT_ERROR(logger, format_string, ...)                                                                           \
  RCLCPP_ERROR(logger, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__, ##__VA_ARGS__)

#define RCOMPONENT_ERROR_NAMED(logger, name, format_string, ...)                                                               \
  RCLCPP_ERROR_NAMED(logger, name, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__,            \
                  ##__VA_ARGS__)

#define RCOMPONENT_ERROR_COND(logger, cond, format_string, ...)                                                                \
  RCLCPP_ERROR_COND(logger, cond, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__,             \
                 ##__VA_ARGS__)

#define RCOMPONENT_ERROR_COND_NAMED(logger, cond, name, format_string, ...)                                                    \
  RCLCPP_ERROR_COND_NAMED(logger, cond, name, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__, \
                       ##__VA_ARGS__)

#define RCOMPONENT_ERROR_ONCE(logger, format_string, ...)                                                                      \
  RCLCPP_ERROR_ONCE(logger, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__, ##__VA_ARGS__)

#define RCOMPONENT_ERROR_ONCE_NAMED(logger, name, format_string, ...)                                                          \
  RCLCPP_ERROR_ONCE_NAMED(logger, name, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__,       \
                       ##__VA_ARGS__)

#define RCOMPONENT_ERROR_THROTTLE(logger, clock, rate, format_string, ...)                                                            \
  RCLCPP_ERROR_THROTTLE(logger, clock, rate, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__,         \
                     ##__VA_ARGS__)

#define RCOMPONENT_ERROR_THROTTLE_NAMED(logger, clock, rate, name, format_string, ...)                                                \
  RCLCPP_ERROR_THROTTLE_NAMED(logger, clock, rate, name, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__,       \
                           __LINE__, ##__VA_ARGS__)

#define RCOMPONENT_ERROR_DELAYED_THROTTLE(logger, rate, format_string, ...)                                                    \
  RCLCPP_ERROR_DELAYED_THROTTLE(logger, rate, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__, \
                             ##__VA_ARGS__)

#define RCOMPONENT_ERROR_DELAYED_THROTTLE_NAMED(logger, rate, name, format_string, ...)                                        \
  RCLCPP_ERROR_DELAYED_THROTTLE_NAMED(logger, rate, name, "%s::%s::%d: " format_string, this->component_name.c_str(),             \
                                   __FUNCTION__, __LINE__, ##__VA_ARGS__)

#define RCOMPONENT_ERROR_FILTER(logger, filter, format_string, ...)                                                            \
  RCLCPP_ERROR_FILTER(logger, filter, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__,         \
                   ##__VA_ARGS__)

#define RCOMPONENT_ERROR_FILTER_NAMED(logger, filter, name, format_string, ...)                                                \
  RCLCPP_ERROR_FILTER_NAMED(logger, filter, name, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__,       \
                         __LINE__, ##__VA_ARGS__)

#define RCOMPONENT_ERROR_STREAM(logger, args)                                                                                  \
  RCLCPP_ERROR_STREAM(logger, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " << args)

#define RCOMPONENT_ERROR_STREAM_NAMED(logger, name, args)                                                                      \
  RCLCPP_ERROR_STREAM_NAMED(logger, name, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " << args)

#define RCOMPONENT_ERROR_STREAM_COND(logger, cond, args)                                                                       \
  RCLCPP_ERROR_STREAM_COND(logger, cond, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " << args)

#define RCOMPONENT_ERROR_STREAM_COND_NAMED(logger, cond, name, args)                                                           \
  RCLCPP_ERROR_STREAM_COND_NAMED(logger, cond, name, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": "     \
                                                               << args)

#define RCOMPONENT_ERROR_STREAM_ONCE(logger, args)                                                                             \
  RCLCPP_ERROR_STREAM_ONCE(logger, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " << args)

#define RCOMPONENT_ERROR_STREAM_ONCE_NAMED                                                                             \
  (logger, name, args)                                                                                                         \
      RCLCPP_ERROR_STREAM_ONCE(logger, name, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " << args)

#define RCOMPONENT_ERROR_STREAM_THROTTLE(logger, clock, rate, args)                                                                   \
  RCLCPP_ERROR_STREAM_THROTTLE(logger, clock, rate, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " << args)

#define RCOMPONENT_ERROR_STREAM_THROTTLE_NAMED(logger, clock, rate, name, args)                                                       \
  RCLCPP_ERROR_STREAM_THROTTLE_NAMED(logger, clock, rate, name, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " \
                                                                   << args)
#define RCOMPONENT_ERROR_STREAM_DELAYED_THROTTLE(logger, rate, args)                                                           \
  define RCLCPP_ERROR_STREAM_DELAYED_THROTTLE(logger, rate, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__      \
                                                                      << ": " << args)
#define RCOMPONENT_ERROR_STREAM_DELAYED_THROTTLE_NAMED(logger, rate, name, args)                                               \
  RCLCPP_ERROR_STREAM_DELAYED_THROTTLE_NAMED(logger, rate, name, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ \
                                                                           << ": " << args)
#define RCOMPONENT_ERROR_STREAM_FILTER(logger, filter, args)                                                                   \
  RCLCPP_ERROR_STREAM_FILTER(logger, filter, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " << args)

#define RCOMPONENT_ERROR_STREAM_FILTER_NAMED(logger, filter, name, args)                                                       \
  RCLCPP_ERROR_STREAM_FILTER_NAMED(logger, filter, name, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " \
                                                                   << args)

// fatal
#define RCOMPONENT_FATAL(logger, format_string, ...)                                                                           \
  RCLCPP_FATAL(logger, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__, ##__VA_ARGS__)

#define RCOMPONENT_FATAL_NAMED(logger, name, format_string, ...)                                                               \
  RCLCPP_FATAL_NAMED(logger, name, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__,            \
                  ##__VA_ARGS__)

#define RCOMPONENT_FATAL_COND(logger, cond, format_string, ...)                                                                \
  RCLCPP_FATAL_COND(logger, cond, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__,             \
                 ##__VA_ARGS__)

#define RCOMPONENT_FATAL_COND_NAMED(logger, cond, name, format_string, ...)                                                    \
  RCLCPP_FATAL_COND_NAMED(logger, cond, name, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__, \
                       ##__VA_ARGS__)

#define RCOMPONENT_FATAL_ONCE(logger, format_string, ...)                                                                      \
  RCLCPP_FATAL_ONCE(logger, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__, ##__VA_ARGS__)

#define RCOMPONENT_FATAL_ONCE_NAMED(logger, name, format_string, ...)                                                          \
  RCLCPP_FATAL_ONCE_NAMED(logger, name, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__,       \
                       ##__VA_ARGS__)

#define RCOMPONENT_FATAL_THROTTLE(logger, rate, format_string, ...)                                                            \
  RCLCPP_FATAL_THROTTLE(logger, rate, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__,         \
                     ##__VA_ARGS__)

#define RCOMPONENT_FATAL_THROTTLE_NAMED(logger, rate, name, format_string, ...)                                                \
  RCLCPP_FATAL_THROTTLE_NAMED(logger, rate, name, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__,       \
                           __LINE__, ##__VA_ARGS__)

#define RCOMPONENT_FATAL_DELAYED_THROTTLE(logger, rate, format_string, ...)                                                    \
  RCLCPP_FATAL_DELAYED_THROTTLE(logger, rate, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__, \
                             ##__VA_ARGS__)

#define RCOMPONENT_FATAL_DELAYED_THROTTLE_NAMED(logger, rate, name, format_string, ...)                                        \
  RCLCPP_FATAL_DELAYED_THROTTLE_NAMED(logger, rate, name, "%s::%s::%d: " format_string, this->component_name.c_str(),             \
                                   __FUNCTION__, __LINE__, ##__VA_ARGS__)

#define RCOMPONENT_FATAL_FILTER(logger, filter, format_string, ...)                                                            \
  RCLCPP_FATAL_FILTER(logger, filter, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__, __LINE__,         \
                   ##__VA_ARGS__)

#define RCOMPONENT_FATAL_FILTER_NAMED(logger, filter, name, format_string, ...)                                                \
  RCLCPP_FATAL_FILTER_NAMED(logger, filter, name, "%s::%s::%d: " format_string, this->component_name.c_str(), __FUNCTION__,       \
                         __LINE__, ##__VA_ARGS__)

#define RCOMPONENT_FATAL_STREAM(logger, args)                                                                                  \
  RCLCPP_FATAL_STREAM(logger, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " << args)

#define RCOMPONENT_FATAL_STREAM_NAMED(logger, name, args)                                                                      \
  RCLCPP_FATAL_STREAM_NAMED(logger, name, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " << args)

#define RCOMPONENT_FATAL_STREAM_COND(logger, cond, args)                                                                       \
  RCLCPP_FATAL_STREAM_COND(logger, cond, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " << args)

#define RCOMPONENT_FATAL_STREAM_COND_NAMED(logger, cond, name, args)                                                           \
  RCLCPP_FATAL_STREAM_COND_NAMED(logger, cond, name, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": "     \
                                                               << args)

#define RCOMPONENT_FATAL_STREAM_ONCE(logger, args)                                                                             \
  RCLCPP_FATAL_STREAM_ONCE(logger, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " << args)

#define RCOMPONENT_FATAL_STREAM_ONCE_NAMED                                                                             \
  (logger, name, args)                                                                                                         \
      RCLCPP_FATAL_STREAM_ONCE(logger, name, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " << args)

#define RCOMPONENT_FATAL_STREAM_THROTTLE(logger, rate, args)                                                                   \
  RCLCPP_FATAL_STREAM_THROTTLE(logger, rate, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " << args)

#define RCOMPONENT_FATAL_STREAM_THROTTLE_NAMED(logger, rate, name, args)                                                       \
  RCLCPP_FATAL_STREAM_THROTTLE_NAMED(logger, rate, name, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " \
                                                                   << args)
#define RCOMPONENT_FATAL_STREAM_DELAYED_THROTTLE(logger, rate, args)                                                           \
  define RCLCPP_FATAL_STREAM_DELAYED_THROTTLE(logger, rate, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__      \
                                                                      << ": " << args)
#define RCOMPONENT_FATAL_STREAM_DELAYED_THROTTLE_NAMED(logger, rate, name, args)                                               \
  RCLCPP_FATAL_STREAM_DELAYED_THROTTLE_NAMED(logger, rate, name, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ \
                                                                           << ": " << args)
#define RCOMPONENT_FATAL_STREAM_FILTER(logger, filter, args)                                                                   \
  RCLCPP_FATAL_STREAM_FILTER(logger, filter, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " << args)

#define RCOMPONENT_FATAL_STREAM_FILTER_NAMED(logger, filter, name, args)                                                       \
  RCLCPP_FATAL_STREAM_FILTER_NAMED(logger, filter, name, this->component_name << "::" << __FUNCTION__ << "::" << __LINE__ << ": " \
                                                                   << args)
#endif  // _RCOMPONENT_LOG_
