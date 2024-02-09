/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to CXX Translator
  File name: PointField_.idl
  Source: PointField_.hpp
  Cyclone DDS: v0.10.2

*****************************************************************/
#ifndef DDSCXX_POINTFIELD__HPP
#define DDSCXX_POINTFIELD__HPP

#include <cstdint>
#include <string>

namespace sensor_msgs {
namespace msg {
namespace dds_ {
namespace PointField_Constants {
const uint8_t INT8_ = 1;

const uint8_t UINT8_ = 2;

const uint8_t INT16_ = 3;

const uint8_t UINT16_ = 4;

const uint8_t INT32_ = 5;

const uint8_t UINT32_ = 6;

const uint8_t FLOAT32_ = 7;

const uint8_t FLOAT64_ = 8;

}  // namespace PointField_Constants

class PointField_ {
private:
  std::string name_;
  uint32_t offset_ = 0;
  uint8_t datatype_ = 0;
  uint32_t count_ = 0;

public:
  PointField_() = default;

  explicit PointField_(const std::string& name, uint32_t offset, uint8_t datatype, uint32_t count)
      : name_(name), offset_(offset), datatype_(datatype), count_(count) {}

  const std::string& name() const { return this->name_; }
  std::string& name() { return this->name_; }
  void name(const std::string& _val_) { this->name_ = _val_; }
  void name(std::string&& _val_) { this->name_ = _val_; }
  uint32_t offset() const { return this->offset_; }
  uint32_t& offset() { return this->offset_; }
  void offset(uint32_t _val_) { this->offset_ = _val_; }
  uint8_t datatype() const { return this->datatype_; }
  uint8_t& datatype() { return this->datatype_; }
  void datatype(uint8_t _val_) { this->datatype_ = _val_; }
  uint32_t count() const { return this->count_; }
  uint32_t& count() { return this->count_; }
  void count(uint32_t _val_) { this->count_ = _val_; }

  bool operator==(const PointField_& _other) const {
    (void)_other;
    return name_ == _other.name_ && offset_ == _other.offset_ && datatype_ == _other.datatype_ &&
           count_ == _other.count_;
  }

  bool operator!=(const PointField_& _other) const { return !(*this == _other); }
};

}  // namespace dds_

}  // namespace msg

}  // namespace sensor_msgs

#include "dds/topic/TopicTraits.hpp"
#include "org/eclipse/cyclonedds/topic/datatopic.hpp"

namespace org {
namespace eclipse {
namespace cyclonedds {
namespace topic {

template <>
constexpr const char* TopicTraits<::sensor_msgs::msg::dds_::PointField_>::getTypeName() {
  return "sensor_msgs::msg::dds_::PointField_";
}

template <>
constexpr bool TopicTraits<::sensor_msgs::msg::dds_::PointField_>::isSelfContained() {
  return false;
}

template <>
constexpr bool TopicTraits<::sensor_msgs::msg::dds_::PointField_>::isKeyless() {
  return true;
}

#ifdef DDSCXX_HAS_TYPE_DISCOVERY
template <>
constexpr unsigned int TopicTraits<::sensor_msgs::msg::dds_::PointField_>::type_map_blob_sz() {
  return 342;
}
template <>
constexpr unsigned int TopicTraits<::sensor_msgs::msg::dds_::PointField_>::type_info_blob_sz() {
  return 100;
}
template <>
inline const uint8_t* TopicTraits<::sensor_msgs::msg::dds_::PointField_>::type_map_blob() {
  static const uint8_t blob[] = {
      0x6b, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0xf1, 0x82, 0x17, 0x10, 0x85, 0xb3, 0xf2,
      0x74, 0x2b, 0xa8, 0x4e, 0xb5, 0xeb, 0x0e, 0x0b, 0x00, 0x53, 0x00, 0x00, 0x00, 0xf1, 0x51,
      0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x43, 0x00, 0x00, 0x00, 0x04,
      0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x70, 0x00,
      0xb0, 0x68, 0x93, 0x1c, 0x0b, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x07,
      0x7a, 0x86, 0xc1, 0x57, 0x00, 0x0b, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00,
      0x02, 0x39, 0x31, 0x10, 0x8d, 0x00, 0x0b, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01,
      0x00, 0x07, 0xe2, 0x94, 0x2a, 0x04, 0x00, 0xbc, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
      0xf2, 0xa3, 0xab, 0xc2, 0x35, 0xd1, 0x3d, 0xdb, 0xab, 0x7a, 0x36, 0x56, 0x16, 0xf1, 0x08,
      0x00, 0xa4, 0x00, 0x00, 0x00, 0xf2, 0x51, 0x01, 0x00, 0x2c, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x24, 0x00, 0x00, 0x00, 0x73, 0x65, 0x6e, 0x73, 0x6f, 0x72, 0x5f, 0x6d, 0x73,
      0x67, 0x73, 0x3a, 0x3a, 0x6d, 0x73, 0x67, 0x3a, 0x3a, 0x64, 0x64, 0x73, 0x5f, 0x3a, 0x3a,
      0x50, 0x6f, 0x69, 0x6e, 0x74, 0x46, 0x69, 0x65, 0x6c, 0x64, 0x5f, 0x00, 0x6c, 0x00, 0x00,
      0x00, 0x04, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
      0x70, 0x00, 0x05, 0x00, 0x00, 0x00, 0x6e, 0x61, 0x6d, 0x65, 0x00, 0x00, 0x00, 0x00, 0x15,
      0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00,
      0x6f, 0x66, 0x66, 0x73, 0x65, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x00, 0x00,
      0x00, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x02, 0x00, 0x09, 0x00, 0x00, 0x00, 0x64, 0x61,
      0x74, 0x61, 0x74, 0x79, 0x70, 0x65, 0x00, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x03,
      0x00, 0x00, 0x00, 0x01, 0x00, 0x07, 0x00, 0x06, 0x00, 0x00, 0x00, 0x63, 0x6f, 0x75, 0x6e,
      0x74, 0x00, 0x00, 0x00, 0x22, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0xf2, 0xa3, 0xab,
      0xc2, 0x35, 0xd1, 0x3d, 0xdb, 0xab, 0x7a, 0x36, 0x56, 0x16, 0xf1, 0x08, 0xf1, 0x82, 0x17,
      0x10, 0x85, 0xb3, 0xf2, 0x74, 0x2b, 0xa8, 0x4e, 0xb5, 0xeb, 0x0e, 0x0b,
  };
  return blob;
}
template <>
inline const uint8_t* TopicTraits<::sensor_msgs::msg::dds_::PointField_>::type_info_blob() {
  static const uint8_t blob[] = {
      0x60, 0x00, 0x00, 0x00, 0x01, 0x10, 0x00, 0x40, 0x28, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00,
      0x00, 0x14, 0x00, 0x00, 0x00, 0xf1, 0x82, 0x17, 0x10, 0x85, 0xb3, 0xf2, 0x74, 0x2b, 0xa8,
      0x4e, 0xb5, 0xeb, 0x0e, 0x0b, 0x00, 0x57, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x10, 0x00, 0x40, 0x28, 0x00, 0x00, 0x00,
      0x24, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0xf2, 0xa3, 0xab, 0xc2, 0x35, 0xd1, 0x3d,
      0xdb, 0xab, 0x7a, 0x36, 0x56, 0x16, 0xf1, 0x08, 0x00, 0xa8, 0x00, 0x00, 0x00, 0x00, 0x00,
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
struct topic_type_name<::sensor_msgs::msg::dds_::PointField_> {
  static std::string value() {
    return org::eclipse::cyclonedds::topic::TopicTraits<
        ::sensor_msgs::msg::dds_::PointField_>::getTypeName();
  }
};

}  // namespace topic
}  // namespace dds

REGISTER_TOPIC_TYPE(::sensor_msgs::msg::dds_::PointField_)

namespace org {
namespace eclipse {
namespace cyclonedds {
namespace core {
namespace cdr {

template <>
propvec& get_type_props<::sensor_msgs::msg::dds_::PointField_>();

template <typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true>
bool write(
    T& streamer,
    const ::sensor_msgs::msg::dds_::PointField_& instance,
    entity_properties_t* props) {
  (void)instance;
  if (!streamer.start_struct(*props)) return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
        if (!streamer.start_member(*prop)) return false;
        if (!write_string(streamer, instance.name(), 0)) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 1:
        if (!streamer.start_member(*prop)) return false;
        if (!write(streamer, instance.offset())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 2:
        if (!streamer.start_member(*prop)) return false;
        if (!write(streamer, instance.datatype())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 3:
        if (!streamer.start_member(*prop)) return false;
        if (!write(streamer, instance.count())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template <typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true>
bool write(S& str, const ::sensor_msgs::msg::dds_::PointField_& instance, bool as_key) {
  auto& props = get_type_props<::sensor_msgs::msg::dds_::PointField_>();
  str.set_mode(cdr_stream::stream_mode::write, as_key);
  return write(str, instance, props.data());
}

template <typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true>
bool read(
    T& streamer, ::sensor_msgs::msg::dds_::PointField_& instance, entity_properties_t* props) {
  (void)instance;
  if (!streamer.start_struct(*props)) return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
        if (!streamer.start_member(*prop)) return false;
        if (!read_string(streamer, instance.name(), 0)) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 1:
        if (!streamer.start_member(*prop)) return false;
        if (!read(streamer, instance.offset())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 2:
        if (!streamer.start_member(*prop)) return false;
        if (!read(streamer, instance.datatype())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 3:
        if (!streamer.start_member(*prop)) return false;
        if (!read(streamer, instance.count())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template <typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true>
bool read(S& str, ::sensor_msgs::msg::dds_::PointField_& instance, bool as_key) {
  auto& props = get_type_props<::sensor_msgs::msg::dds_::PointField_>();
  str.set_mode(cdr_stream::stream_mode::read, as_key);
  return read(str, instance, props.data());
}

template <typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true>
bool move(
    T& streamer,
    const ::sensor_msgs::msg::dds_::PointField_& instance,
    entity_properties_t* props) {
  (void)instance;
  if (!streamer.start_struct(*props)) return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
        if (!streamer.start_member(*prop)) return false;
        if (!move_string(streamer, instance.name(), 0)) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 1:
        if (!streamer.start_member(*prop)) return false;
        if (!move(streamer, instance.offset())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 2:
        if (!streamer.start_member(*prop)) return false;
        if (!move(streamer, instance.datatype())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 3:
        if (!streamer.start_member(*prop)) return false;
        if (!move(streamer, instance.count())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template <typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true>
bool move(S& str, const ::sensor_msgs::msg::dds_::PointField_& instance, bool as_key) {
  auto& props = get_type_props<::sensor_msgs::msg::dds_::PointField_>();
  str.set_mode(cdr_stream::stream_mode::move, as_key);
  return move(str, instance, props.data());
}

template <typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true>
bool max(
    T& streamer,
    const ::sensor_msgs::msg::dds_::PointField_& instance,
    entity_properties_t* props) {
  (void)instance;
  if (!streamer.start_struct(*props)) return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
        if (!streamer.start_member(*prop)) return false;
        if (!max_string(streamer, instance.name(), 0)) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 1:
        if (!streamer.start_member(*prop)) return false;
        if (!max(streamer, instance.offset())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 2:
        if (!streamer.start_member(*prop)) return false;
        if (!max(streamer, instance.datatype())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 3:
        if (!streamer.start_member(*prop)) return false;
        if (!max(streamer, instance.count())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template <typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true>
bool max(S& str, const ::sensor_msgs::msg::dds_::PointField_& instance, bool as_key) {
  auto& props = get_type_props<::sensor_msgs::msg::dds_::PointField_>();
  str.set_mode(cdr_stream::stream_mode::max, as_key);
  return max(str, instance, props.data());
}

}  // namespace cdr
}  // namespace core
}  // namespace cyclonedds
}  // namespace eclipse
}  // namespace org

#endif  // DDSCXX_POINTFIELD__HPP
