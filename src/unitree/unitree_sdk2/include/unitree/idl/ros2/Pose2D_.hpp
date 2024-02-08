/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to CXX Translator
  File name: Pose2D_.idl
  Source: Pose2D_.hpp
  Cyclone DDS: v0.10.2

*****************************************************************/
#ifndef DDSCXX_POSE2D__HPP
#define DDSCXX_POSE2D__HPP


namespace geometry_msgs
{
namespace msg
{
namespace dds_
{
class Pose2D_
{
private:
 double x_ = 0.0;
 double y_ = 0.0;
 double theta_ = 0.0;

public:
  Pose2D_() = default;

  explicit Pose2D_(
    double x,
    double y,
    double theta) :
    x_(x),
    y_(y),
    theta_(theta) { }

  double x() const { return this->x_; }
  double& x() { return this->x_; }
  void x(double _val_) { this->x_ = _val_; }
  double y() const { return this->y_; }
  double& y() { return this->y_; }
  void y(double _val_) { this->y_ = _val_; }
  double theta() const { return this->theta_; }
  double& theta() { return this->theta_; }
  void theta(double _val_) { this->theta_ = _val_; }

  bool operator==(const Pose2D_& _other) const
  {
    (void) _other;
    return x_ == _other.x_ &&
      y_ == _other.y_ &&
      theta_ == _other.theta_;
  }

  bool operator!=(const Pose2D_& _other) const
  {
    return !(*this == _other);
  }

};

}

}

}

#include "dds/topic/TopicTraits.hpp"
#include "org/eclipse/cyclonedds/topic/datatopic.hpp"

namespace org {
namespace eclipse {
namespace cyclonedds {
namespace topic {

template <> constexpr const char* TopicTraits<::geometry_msgs::msg::dds_::Pose2D_>::getTypeName()
{
  return "geometry_msgs::msg::dds_::Pose2D_";
}

template <> constexpr bool TopicTraits<::geometry_msgs::msg::dds_::Pose2D_>::isKeyless()
{
  return true;
}

#ifdef DDSCXX_HAS_TYPE_DISCOVERY
template<> constexpr unsigned int TopicTraits<::geometry_msgs::msg::dds_::Pose2D_>::type_map_blob_sz() { return 286; }
template<> constexpr unsigned int TopicTraits<::geometry_msgs::msg::dds_::Pose2D_>::type_info_blob_sz() { return 100; }
template<> inline const uint8_t * TopicTraits<::geometry_msgs::msg::dds_::Pose2D_>::type_map_blob() {
  static const uint8_t blob[] = {
 0x5b,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0xf1,  0xe6,  0x4d,  0xe9,  0xc3,  0xbd,  0x87,  0xa5, 
 0x1f,  0x57,  0xf7,  0x59,  0xf0,  0x59,  0x77,  0x00,  0x43,  0x00,  0x00,  0x00,  0xf1,  0x51,  0x01,  0x00, 
 0x01,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x33,  0x00,  0x00,  0x00,  0x03,  0x00,  0x00,  0x00, 
 0x0b,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x01,  0x00,  0x0a,  0x9d,  0xd4,  0xe4,  0x61,  0x00, 
 0x0b,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0x0a,  0x41,  0x52,  0x90,  0x76,  0x00, 
 0x0b,  0x00,  0x00,  0x00,  0x02,  0x00,  0x00,  0x00,  0x01,  0x00,  0x0a,  0x61,  0xa7,  0x4b,  0xe6,  0x00, 
 0x94,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0xf2,  0xc6,  0x23,  0x0c,  0x9d,  0x1e,  0x0d,  0x38, 
 0x95,  0xf0,  0x39,  0x06,  0x50,  0x19,  0xc5,  0x00,  0x7c,  0x00,  0x00,  0x00,  0xf2,  0x51,  0x01,  0x00, 
 0x2a,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x22,  0x00,  0x00,  0x00,  0x67,  0x65,  0x6f,  0x6d, 
 0x65,  0x74,  0x72,  0x79,  0x5f,  0x6d,  0x73,  0x67,  0x73,  0x3a,  0x3a,  0x6d,  0x73,  0x67,  0x3a,  0x3a, 
 0x64,  0x64,  0x73,  0x5f,  0x3a,  0x3a,  0x50,  0x6f,  0x73,  0x65,  0x32,  0x44,  0x5f,  0x00,  0x00,  0x00, 
 0x44,  0x00,  0x00,  0x00,  0x03,  0x00,  0x00,  0x00,  0x10,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x0a,  0x00,  0x02,  0x00,  0x00,  0x00,  0x78,  0x00,  0x00,  0x00,  0x10,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0x0a,  0x00,  0x02,  0x00,  0x00,  0x00,  0x79,  0x00,  0x00,  0x00, 
 0x14,  0x00,  0x00,  0x00,  0x02,  0x00,  0x00,  0x00,  0x01,  0x00,  0x0a,  0x00,  0x06,  0x00,  0x00,  0x00, 
 0x74,  0x68,  0x65,  0x74,  0x61,  0x00,  0x00,  0x00,  0x22,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00, 
 0xf2,  0xc6,  0x23,  0x0c,  0x9d,  0x1e,  0x0d,  0x38,  0x95,  0xf0,  0x39,  0x06,  0x50,  0x19,  0xc5,  0xf1, 
 0xe6,  0x4d,  0xe9,  0xc3,  0xbd,  0x87,  0xa5,  0x1f,  0x57,  0xf7,  0x59,  0xf0,  0x59,  0x77, };
  return blob;
}
template<> inline const uint8_t * TopicTraits<::geometry_msgs::msg::dds_::Pose2D_>::type_info_blob() {
  static const uint8_t blob[] = {
 0x60,  0x00,  0x00,  0x00,  0x01,  0x10,  0x00,  0x40,  0x28,  0x00,  0x00,  0x00,  0x24,  0x00,  0x00,  0x00, 
 0x14,  0x00,  0x00,  0x00,  0xf1,  0xe6,  0x4d,  0xe9,  0xc3,  0xbd,  0x87,  0xa5,  0x1f,  0x57,  0xf7,  0x59, 
 0xf0,  0x59,  0x77,  0x00,  0x47,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x04,  0x00,  0x00,  0x00, 
 0x00,  0x00,  0x00,  0x00,  0x02,  0x10,  0x00,  0x40,  0x28,  0x00,  0x00,  0x00,  0x24,  0x00,  0x00,  0x00, 
 0x14,  0x00,  0x00,  0x00,  0xf2,  0xc6,  0x23,  0x0c,  0x9d,  0x1e,  0x0d,  0x38,  0x95,  0xf0,  0x39,  0x06, 
 0x50,  0x19,  0xc5,  0x00,  0x80,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x04,  0x00,  0x00,  0x00, 
 0x00,  0x00,  0x00,  0x00, };
  return blob;
}
#endif //DDSCXX_HAS_TYPE_DISCOVERY

} //namespace topic
} //namespace cyclonedds
} //namespace eclipse
} //namespace org

namespace dds {
namespace topic {

template <>
struct topic_type_name<::geometry_msgs::msg::dds_::Pose2D_>
{
    static std::string value()
    {
      return org::eclipse::cyclonedds::topic::TopicTraits<::geometry_msgs::msg::dds_::Pose2D_>::getTypeName();
    }
};

}
}

REGISTER_TOPIC_TYPE(::geometry_msgs::msg::dds_::Pose2D_)

namespace org{
namespace eclipse{
namespace cyclonedds{
namespace core{
namespace cdr{

template<>
propvec &get_type_props<::geometry_msgs::msg::dds_::Pose2D_>();

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool write(T& streamer, const ::geometry_msgs::msg::dds_::Pose2D_& instance, entity_properties_t *props) {
  (void)instance;
  if (!streamer.start_struct(*props))
    return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
      if (!streamer.start_member(*prop))
        return false;
      if (!write(streamer, instance.x()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 1:
      if (!streamer.start_member(*prop))
        return false;
      if (!write(streamer, instance.y()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 2:
      if (!streamer.start_member(*prop))
        return false;
      if (!write(streamer, instance.theta()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool write(S& str, const ::geometry_msgs::msg::dds_::Pose2D_& instance, bool as_key) {
  auto &props = get_type_props<::geometry_msgs::msg::dds_::Pose2D_>();
  str.set_mode(cdr_stream::stream_mode::write, as_key);
  return write(str, instance, props.data()); 
}

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool read(T& streamer, ::geometry_msgs::msg::dds_::Pose2D_& instance, entity_properties_t *props) {
  (void)instance;
  if (!streamer.start_struct(*props))
    return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
      if (!streamer.start_member(*prop))
        return false;
      if (!read(streamer, instance.x()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 1:
      if (!streamer.start_member(*prop))
        return false;
      if (!read(streamer, instance.y()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 2:
      if (!streamer.start_member(*prop))
        return false;
      if (!read(streamer, instance.theta()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool read(S& str, ::geometry_msgs::msg::dds_::Pose2D_& instance, bool as_key) {
  auto &props = get_type_props<::geometry_msgs::msg::dds_::Pose2D_>();
  str.set_mode(cdr_stream::stream_mode::read, as_key);
  return read(str, instance, props.data()); 
}

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool move(T& streamer, const ::geometry_msgs::msg::dds_::Pose2D_& instance, entity_properties_t *props) {
  (void)instance;
  if (!streamer.start_struct(*props))
    return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
      if (!streamer.start_member(*prop))
        return false;
      if (!move(streamer, instance.x()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 1:
      if (!streamer.start_member(*prop))
        return false;
      if (!move(streamer, instance.y()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 2:
      if (!streamer.start_member(*prop))
        return false;
      if (!move(streamer, instance.theta()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool move(S& str, const ::geometry_msgs::msg::dds_::Pose2D_& instance, bool as_key) {
  auto &props = get_type_props<::geometry_msgs::msg::dds_::Pose2D_>();
  str.set_mode(cdr_stream::stream_mode::move, as_key);
  return move(str, instance, props.data()); 
}

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool max(T& streamer, const ::geometry_msgs::msg::dds_::Pose2D_& instance, entity_properties_t *props) {
  (void)instance;
  if (!streamer.start_struct(*props))
    return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
      if (!streamer.start_member(*prop))
        return false;
      if (!max(streamer, instance.x()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 1:
      if (!streamer.start_member(*prop))
        return false;
      if (!max(streamer, instance.y()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 2:
      if (!streamer.start_member(*prop))
        return false;
      if (!max(streamer, instance.theta()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool max(S& str, const ::geometry_msgs::msg::dds_::Pose2D_& instance, bool as_key) {
  auto &props = get_type_props<::geometry_msgs::msg::dds_::Pose2D_>();
  str.set_mode(cdr_stream::stream_mode::max, as_key);
  return max(str, instance, props.data()); 
}

} //namespace cdr
} //namespace core
} //namespace cyclonedds
} //namespace eclipse
} //namespace org

#endif // DDSCXX_POSE2D__HPP
