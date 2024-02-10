/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to CXX Translator
  File name: Point32_.idl
  Source: Point32_.hpp
  Cyclone DDS: v0.10.2

*****************************************************************/
#ifndef DDSCXX_POINT32__HPP
#define DDSCXX_POINT32__HPP

namespace geometry_msgs {
namespace msg {
namespace dds_ {
class Point32_ {
private:
  float x_ = 0.0f;
  float y_ = 0.0f;
  float z_ = 0.0f;

public:
  Point32_() = default;

  explicit Point32_(float x, float y, float z) : x_(x), y_(y), z_(z) {}

  float x() const { return this->x_; }
  float& x() { return this->x_; }
  void x(float _val_) { this->x_ = _val_; }
  float y() const { return this->y_; }
  float& y() { return this->y_; }
  void y(float _val_) { this->y_ = _val_; }
  float z() const { return this->z_; }
  float& z() { return this->z_; }
  void z(float _val_) { this->z_ = _val_; }

  bool operator==(const Point32_& _other) const {
    (void)_other;
    return x_ == _other.x_ && y_ == _other.y_ && z_ == _other.z_;
  }

  bool operator!=(const Point32_& _other) const { return !(*this == _other); }
};

}  // namespace dds_

}  // namespace msg

}  // namespace geometry_msgs

#include "dds/topic/TopicTraits.hpp"
#include "org/eclipse/cyclonedds/topic/datatopic.hpp"

namespace org {
namespace eclipse {
namespace cyclonedds {
namespace topic {

template <>
constexpr const char* TopicTraits<::geometry_msgs::msg::dds_::Point32_>::getTypeName() {
  return "geometry_msgs::msg::dds_::Point32_";
}

template <>
constexpr bool TopicTraits<::geometry_msgs::msg::dds_::Point32_>::isKeyless() {
  return true;
}

#ifdef DDSCXX_HAS_TYPE_DISCOVERY
template <>
constexpr unsigned int TopicTraits<::geometry_msgs::msg::dds_::Point32_>::type_map_blob_sz() {
  return 282;
}
template <>
constexpr unsigned int TopicTraits<::geometry_msgs::msg::dds_::Point32_>::type_info_blob_sz() {
  return 100;
}
template <>
inline const uint8_t* TopicTraits<::geometry_msgs::msg::dds_::Point32_>::type_map_blob() {
  static const uint8_t blob[] = {
      0x5b, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0xf1, 0xbd, 0xb3, 0x4b, 0x07, 0x97, 0x70,
      0xd5, 0x44, 0x4c, 0xda, 0x2d, 0x20, 0xb7, 0x83, 0x00, 0x43, 0x00, 0x00, 0x00, 0xf1, 0x51,
      0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x33, 0x00, 0x00, 0x00, 0x03,
      0x00, 0x00, 0x00, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x09, 0x9d,
      0xd4, 0xe4, 0x61, 0x00, 0x0b, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x09,
      0x41, 0x52, 0x90, 0x76, 0x00, 0x0b, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00,
      0x09, 0xfb, 0xad, 0xe9, 0xe3, 0x00, 0x90, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0xf2,
      0xb5, 0xc0, 0x1f, 0x51, 0xb0, 0x99, 0xa4, 0x54, 0xb3, 0xb8, 0xff, 0x68, 0x29, 0xe5, 0x00,
      0x78, 0x00, 0x00, 0x00, 0xf2, 0x51, 0x01, 0x00, 0x2b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x23, 0x00, 0x00, 0x00, 0x67, 0x65, 0x6f, 0x6d, 0x65, 0x74, 0x72, 0x79, 0x5f, 0x6d,
      0x73, 0x67, 0x73, 0x3a, 0x3a, 0x6d, 0x73, 0x67, 0x3a, 0x3a, 0x64, 0x64, 0x73, 0x5f, 0x3a,
      0x3a, 0x50, 0x6f, 0x69, 0x6e, 0x74, 0x33, 0x32, 0x5f, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00,
      0x03, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x09,
      0x00, 0x02, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x01, 0x00,
      0x00, 0x00, 0x01, 0x00, 0x09, 0x00, 0x02, 0x00, 0x00, 0x00, 0x79, 0x00, 0x00, 0x00, 0x10,
      0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x09, 0x00, 0x02, 0x00, 0x00, 0x00,
      0x7a, 0x00, 0x00, 0x00, 0x22, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0xf2, 0xb5, 0xc0,
      0x1f, 0x51, 0xb0, 0x99, 0xa4, 0x54, 0xb3, 0xb8, 0xff, 0x68, 0x29, 0xe5, 0xf1, 0xbd, 0xb3,
      0x4b, 0x07, 0x97, 0x70, 0xd5, 0x44, 0x4c, 0xda, 0x2d, 0x20, 0xb7, 0x83,
  };
  return blob;
}
template <>
inline const uint8_t* TopicTraits<::geometry_msgs::msg::dds_::Point32_>::type_info_blob() {
  static const uint8_t blob[] = {
      0x60, 0x00, 0x00, 0x00, 0x01, 0x10, 0x00, 0x40, 0x28, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00,
      0x00, 0x14, 0x00, 0x00, 0x00, 0xf1, 0xbd, 0xb3, 0x4b, 0x07, 0x97, 0x70, 0xd5, 0x44, 0x4c,
      0xda, 0x2d, 0x20, 0xb7, 0x83, 0x00, 0x47, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x10, 0x00, 0x40, 0x28, 0x00, 0x00, 0x00,
      0x24, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0xf2, 0xb5, 0xc0, 0x1f, 0x51, 0xb0, 0x99,
      0xa4, 0x54, 0xb3, 0xb8, 0xff, 0x68, 0x29, 0xe5, 0x00, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  };
  return blob;
}
#endif  // DDSCXX_HAS_TYPE_DISCOVERY

}  // namespace topic
}  // namespace cyclonedds
}  // namespace eclipse
}  // namespace org

namespace dds {
namespace topic {

template <>
struct topic_type_name<::geometry_msgs::msg::dds_::Point32_> {
  static std::string value() {
    return org::eclipse::cyclonedds::topic::TopicTraits<
        ::geometry_msgs::msg::dds_::Point32_>::getTypeName();
  }
};

}  // namespace topic
}  // namespace dds

REGISTER_TOPIC_TYPE(::geometry_msgs::msg::dds_::Point32_)

namespace org {
namespace eclipse {
namespace cyclonedds {
namespace core {
namespace cdr {

template <>
propvec& get_type_props<::geometry_msgs::msg::dds_::Point32_>();

template <typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true>
bool write(
    T& streamer, const ::geometry_msgs::msg::dds_::Point32_& instance, entity_properties_t* props) {
  (void)instance;
  if (!streamer.start_struct(*props)) return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
        if (!streamer.start_member(*prop)) return false;
        if (!write(streamer, instance.x())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 1:
        if (!streamer.start_member(*prop)) return false;
        if (!write(streamer, instance.y())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 2:
        if (!streamer.start_member(*prop)) return false;
        if (!write(streamer, instance.z())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template <typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true>
bool write(S& str, const ::geometry_msgs::msg::dds_::Point32_& instance, bool as_key) {
  auto& props = get_type_props<::geometry_msgs::msg::dds_::Point32_>();
  str.set_mode(cdr_stream::stream_mode::write, as_key);
  return write(str, instance, props.data());
}

template <typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true>
bool read(T& streamer, ::geometry_msgs::msg::dds_::Point32_& instance, entity_properties_t* props) {
  (void)instance;
  if (!streamer.start_struct(*props)) return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
        if (!streamer.start_member(*prop)) return false;
        if (!read(streamer, instance.x())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 1:
        if (!streamer.start_member(*prop)) return false;
        if (!read(streamer, instance.y())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 2:
        if (!streamer.start_member(*prop)) return false;
        if (!read(streamer, instance.z())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template <typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true>
bool read(S& str, ::geometry_msgs::msg::dds_::Point32_& instance, bool as_key) {
  auto& props = get_type_props<::geometry_msgs::msg::dds_::Point32_>();
  str.set_mode(cdr_stream::stream_mode::read, as_key);
  return read(str, instance, props.data());
}

template <typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true>
bool move(
    T& streamer, const ::geometry_msgs::msg::dds_::Point32_& instance, entity_properties_t* props) {
  (void)instance;
  if (!streamer.start_struct(*props)) return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
        if (!streamer.start_member(*prop)) return false;
        if (!move(streamer, instance.x())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 1:
        if (!streamer.start_member(*prop)) return false;
        if (!move(streamer, instance.y())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 2:
        if (!streamer.start_member(*prop)) return false;
        if (!move(streamer, instance.z())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template <typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true>
bool move(S& str, const ::geometry_msgs::msg::dds_::Point32_& instance, bool as_key) {
  auto& props = get_type_props<::geometry_msgs::msg::dds_::Point32_>();
  str.set_mode(cdr_stream::stream_mode::move, as_key);
  return move(str, instance, props.data());
}

template <typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true>
bool max(
    T& streamer, const ::geometry_msgs::msg::dds_::Point32_& instance, entity_properties_t* props) {
  (void)instance;
  if (!streamer.start_struct(*props)) return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
        if (!streamer.start_member(*prop)) return false;
        if (!max(streamer, instance.x())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 1:
        if (!streamer.start_member(*prop)) return false;
        if (!max(streamer, instance.y())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 2:
        if (!streamer.start_member(*prop)) return false;
        if (!max(streamer, instance.z())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template <typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true>
bool max(S& str, const ::geometry_msgs::msg::dds_::Point32_& instance, bool as_key) {
  auto& props = get_type_props<::geometry_msgs::msg::dds_::Point32_>();
  str.set_mode(cdr_stream::stream_mode::max, as_key);
  return max(str, instance, props.data());
}

}  // namespace cdr
}  // namespace core
}  // namespace cyclonedds
}  // namespace eclipse
}  // namespace org

#endif  // DDSCXX_POINT32__HPP