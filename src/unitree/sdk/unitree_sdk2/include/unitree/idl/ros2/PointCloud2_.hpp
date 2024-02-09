/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to CXX Translator
  File name: PointCloud2_.idl
  Source: PointCloud2_.hpp
  Cyclone DDS: v0.10.2

*****************************************************************/
#ifndef DDSCXX_POINTCLOUD2__HPP
#define DDSCXX_POINTCLOUD2__HPP

#include "PointField_.hpp"

#include "Header_.hpp"

#include <cstdint>
#include <vector>

namespace sensor_msgs {
namespace msg {
namespace dds_ {
class PointCloud2_ {
private:
  ::std_msgs::msg::dds_::Header_ header_;
  uint32_t height_ = 0;
  uint32_t width_ = 0;
  std::vector<::sensor_msgs::msg::dds_::PointField_> fields_;
  bool is_bigendian_ = false;
  uint32_t point_step_ = 0;
  uint32_t row_step_ = 0;
  std::vector<uint8_t> data_;
  bool is_dense_ = false;

public:
  PointCloud2_() = default;

  explicit PointCloud2_(
      const ::std_msgs::msg::dds_::Header_& header,
      uint32_t height,
      uint32_t width,
      const std::vector<::sensor_msgs::msg::dds_::PointField_>& fields,
      bool is_bigendian,
      uint32_t point_step,
      uint32_t row_step,
      const std::vector<uint8_t>& data,
      bool is_dense)
      : header_(header),
        height_(height),
        width_(width),
        fields_(fields),
        is_bigendian_(is_bigendian),
        point_step_(point_step),
        row_step_(row_step),
        data_(data),
        is_dense_(is_dense) {}

  const ::std_msgs::msg::dds_::Header_& header() const { return this->header_; }
  ::std_msgs::msg::dds_::Header_& header() { return this->header_; }
  void header(const ::std_msgs::msg::dds_::Header_& _val_) { this->header_ = _val_; }
  void header(::std_msgs::msg::dds_::Header_&& _val_) { this->header_ = _val_; }
  uint32_t height() const { return this->height_; }
  uint32_t& height() { return this->height_; }
  void height(uint32_t _val_) { this->height_ = _val_; }
  uint32_t width() const { return this->width_; }
  uint32_t& width() { return this->width_; }
  void width(uint32_t _val_) { this->width_ = _val_; }
  const std::vector<::sensor_msgs::msg::dds_::PointField_>& fields() const { return this->fields_; }
  std::vector<::sensor_msgs::msg::dds_::PointField_>& fields() { return this->fields_; }
  void fields(const std::vector<::sensor_msgs::msg::dds_::PointField_>& _val_) {
    this->fields_ = _val_;
  }
  void fields(std::vector<::sensor_msgs::msg::dds_::PointField_>&& _val_) { this->fields_ = _val_; }
  bool is_bigendian() const { return this->is_bigendian_; }
  bool& is_bigendian() { return this->is_bigendian_; }
  void is_bigendian(bool _val_) { this->is_bigendian_ = _val_; }
  uint32_t point_step() const { return this->point_step_; }
  uint32_t& point_step() { return this->point_step_; }
  void point_step(uint32_t _val_) { this->point_step_ = _val_; }
  uint32_t row_step() const { return this->row_step_; }
  uint32_t& row_step() { return this->row_step_; }
  void row_step(uint32_t _val_) { this->row_step_ = _val_; }
  const std::vector<uint8_t>& data() const { return this->data_; }
  std::vector<uint8_t>& data() { return this->data_; }
  void data(const std::vector<uint8_t>& _val_) { this->data_ = _val_; }
  void data(std::vector<uint8_t>&& _val_) { this->data_ = _val_; }
  bool is_dense() const { return this->is_dense_; }
  bool& is_dense() { return this->is_dense_; }
  void is_dense(bool _val_) { this->is_dense_ = _val_; }

  bool operator==(const PointCloud2_& _other) const {
    (void)_other;
    return header_ == _other.header_ && height_ == _other.height_ && width_ == _other.width_ &&
           fields_ == _other.fields_ && is_bigendian_ == _other.is_bigendian_ &&
           point_step_ == _other.point_step_ && row_step_ == _other.row_step_ &&
           data_ == _other.data_ && is_dense_ == _other.is_dense_;
  }

  bool operator!=(const PointCloud2_& _other) const { return !(*this == _other); }
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
constexpr const char* TopicTraits<::sensor_msgs::msg::dds_::PointCloud2_>::getTypeName() {
  return "sensor_msgs::msg::dds_::PointCloud2_";
}

template <>
constexpr bool TopicTraits<::sensor_msgs::msg::dds_::PointCloud2_>::isSelfContained() {
  return false;
}

template <>
constexpr bool TopicTraits<::sensor_msgs::msg::dds_::PointCloud2_>::isKeyless() {
  return true;
}

#ifdef DDSCXX_HAS_TYPE_DISCOVERY
template <>
constexpr unsigned int TopicTraits<::sensor_msgs::msg::dds_::PointCloud2_>::type_map_blob_sz() {
  return 1472;
}
template <>
constexpr unsigned int TopicTraits<::sensor_msgs::msg::dds_::PointCloud2_>::type_info_blob_sz() {
  return 244;
}
template <>
inline const uint8_t* TopicTraits<::sensor_msgs::msg::dds_::PointCloud2_>::type_map_blob() {
  static const uint8_t blob[] = {
      0xeb, 0x01, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0xf1, 0x7f, 0x62, 0xa4, 0x8e, 0xfc, 0x63,
      0xa8, 0xa6, 0xfe, 0x5f, 0x9e, 0xda, 0x06, 0xd7, 0x00, 0xcb, 0x00, 0x00, 0x00, 0xf1, 0x51,
      0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xbb, 0x00, 0x00, 0x00, 0x09,
      0x00, 0x00, 0x00, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0xf1, 0xdc,
      0xf1, 0x2c, 0xd2, 0xdd, 0x5e, 0x71, 0x2c, 0xb7, 0xb1, 0xe5, 0x1f, 0xa3, 0xf2, 0x09, 0x9f,
      0xb9, 0x95, 0x00, 0x00, 0x00, 0x0b, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,
      0x07, 0xb4, 0x35, 0xe2, 0x27, 0x00, 0x0b, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01,
      0x00, 0x07, 0xea, 0xae, 0x26, 0xa6, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00,
      0x01, 0x00, 0x80, 0xf1, 0x01, 0x00, 0x00, 0xf1, 0x82, 0x17, 0x10, 0x85, 0xb3, 0xf2, 0x74,
      0x2b, 0xa8, 0x4e, 0xb5, 0xeb, 0x0e, 0x0b, 0xd0, 0x5b, 0x6e, 0xd7, 0x00, 0x00, 0x0b, 0x00,
      0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x70, 0x8e, 0x8a, 0xc3, 0x00, 0x0b,
      0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x01, 0x00, 0x07, 0x1a, 0x99, 0x6a, 0x87, 0x00,
      0x0b, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x01, 0x00, 0x07, 0x8b, 0x4f, 0x6f, 0x6b,
      0x00, 0x10, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x01, 0x00, 0x80, 0xf3, 0x01, 0x00,
      0x00, 0x02, 0x8d, 0x77, 0x7f, 0x38, 0x0b, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01,
      0x00, 0x01, 0x40, 0x23, 0x8a, 0xea, 0xf1, 0xdc, 0xf1, 0x2c, 0xd2, 0xdd, 0x5e, 0x71, 0x2c,
      0xb7, 0xb1, 0xe5, 0x1f, 0xa3, 0xf2, 0x00, 0x00, 0x44, 0x00, 0x00, 0x00, 0xf1, 0x51, 0x01,
      0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x34, 0x00, 0x00, 0x00, 0x02, 0x00,
      0x00, 0x00, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0xf1, 0x56, 0x7c,
      0x5a, 0x93, 0x54, 0x1c, 0x3b, 0x10, 0x86, 0xa4, 0xba, 0x46, 0xf9, 0x8d, 0x96, 0xb8, 0xc7,
      0x8d, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x70,
      0x00, 0x4b, 0xb3, 0x9c, 0x5c, 0xf1, 0x56, 0x7c, 0x5a, 0x93, 0x54, 0x1c, 0x3b, 0x10, 0x86,
      0xa4, 0xba, 0x46, 0xf9, 0x8d, 0x00, 0x33, 0x00, 0x00, 0x00, 0xf1, 0x51, 0x01, 0x00, 0x01,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
      0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x04, 0x74, 0x45, 0x9c, 0xa3,
      0x00, 0x0b, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x07, 0xe2, 0x04, 0x64,
      0xd5, 0xf1, 0x82, 0x17, 0x10, 0x85, 0xb3, 0xf2, 0x74, 0x2b, 0xa8, 0x4e, 0xb5, 0xeb, 0x0e,
      0x0b, 0x00, 0x00, 0x53, 0x00, 0x00, 0x00, 0xf1, 0x51, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x43, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x70, 0x00, 0xb0, 0x68, 0x93, 0x1c, 0x0b, 0x00,
      0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x07, 0x7a, 0x86, 0xc1, 0x57, 0x00, 0x0b,
      0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x02, 0x39, 0x31, 0x10, 0x8d, 0x00,
      0x0b, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x07, 0xe2, 0x94, 0x2a, 0x04,
      0x00, 0x4c, 0x03, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0xf2, 0xf6, 0xf1, 0xa7, 0xb4, 0x13,
      0x2f, 0x17, 0xb4, 0xae, 0x1d, 0x73, 0x77, 0xc5, 0xbc, 0x00, 0x63, 0x01, 0x00, 0x00, 0xf2,
      0x51, 0x01, 0x00, 0x2d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x25, 0x00, 0x00, 0x00,
      0x73, 0x65, 0x6e, 0x73, 0x6f, 0x72, 0x5f, 0x6d, 0x73, 0x67, 0x73, 0x3a, 0x3a, 0x6d, 0x73,
      0x67, 0x3a, 0x3a, 0x64, 0x64, 0x73, 0x5f, 0x3a, 0x3a, 0x50, 0x6f, 0x69, 0x6e, 0x74, 0x43,
      0x6c, 0x6f, 0x75, 0x64, 0x32, 0x5f, 0x00, 0x00, 0x00, 0x00, 0x27, 0x01, 0x00, 0x00, 0x09,
      0x00, 0x00, 0x00, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0xf2, 0xe5,
      0x76, 0x5e, 0xc4, 0x8c, 0xff, 0xd4, 0x19, 0xed, 0x7f, 0xe8, 0x4e, 0x2a, 0x55, 0x00, 0x00,
      0x00, 0x07, 0x00, 0x00, 0x00, 0x68, 0x65, 0x61, 0x64, 0x65, 0x72, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x15, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x07, 0x00, 0x07,
      0x00, 0x00, 0x00, 0x68, 0x65, 0x69, 0x67, 0x68, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x14, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x07, 0x00, 0x06, 0x00, 0x00,
      0x00, 0x77, 0x69, 0x64, 0x74, 0x68, 0x00, 0x00, 0x00, 0x29, 0x00, 0x00, 0x00, 0x03, 0x00,
      0x00, 0x00, 0x01, 0x00, 0x80, 0xf2, 0x01, 0x00, 0x00, 0xf2, 0xa3, 0xab, 0xc2, 0x35, 0xd1,
      0x3d, 0xdb, 0xab, 0x7a, 0x36, 0x56, 0x16, 0xf1, 0x08, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00,
      0x66, 0x69, 0x65, 0x6c, 0x64, 0x73, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b, 0x00, 0x00,
      0x00, 0x04, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x0d, 0x00, 0x00, 0x00, 0x69, 0x73,
      0x5f, 0x62, 0x69, 0x67, 0x65, 0x6e, 0x64, 0x69, 0x61, 0x6e, 0x00, 0x00, 0x00, 0x00, 0x19,
      0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x01, 0x00, 0x07, 0x00, 0x0b, 0x00, 0x00, 0x00,
      0x70, 0x6f, 0x69, 0x6e, 0x74, 0x5f, 0x73, 0x74, 0x65, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x17, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x01, 0x00, 0x07, 0x00, 0x09, 0x00,
      0x00, 0x00, 0x72, 0x6f, 0x77, 0x5f, 0x73, 0x74, 0x65, 0x70, 0x00, 0x00, 0x00, 0x00, 0x17,
      0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x01, 0x00, 0x80, 0xf3, 0x01, 0x00, 0x00, 0x02,
      0x05, 0x00, 0x00, 0x00, 0x64, 0x61, 0x74, 0x61, 0x00, 0x00, 0x00, 0x00, 0x17, 0x00, 0x00,
      0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x09, 0x00, 0x00, 0x00, 0x69, 0x73,
      0x5f, 0x64, 0x65, 0x6e, 0x73, 0x65, 0x00, 0x00, 0x00, 0xf2, 0xe5, 0x76, 0x5e, 0xc4, 0x8c,
      0xff, 0xd4, 0x19, 0xed, 0x7f, 0xe8, 0x4e, 0x2a, 0x55, 0x00, 0x00, 0x7b, 0x00, 0x00, 0x00,
      0xf2, 0x51, 0x01, 0x00, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1d, 0x00, 0x00,
      0x00, 0x73, 0x74, 0x64, 0x5f, 0x6d, 0x73, 0x67, 0x73, 0x3a, 0x3a, 0x6d, 0x73, 0x67, 0x3a,
      0x3a, 0x64, 0x64, 0x73, 0x5f, 0x3a, 0x3a, 0x48, 0x65, 0x61, 0x64, 0x65, 0x72, 0x5f, 0x00,
      0x00, 0x00, 0x00, 0x47, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0xf2, 0xd4, 0x85, 0x4f, 0x13, 0xae, 0xf3, 0x2d, 0xfe,
      0x21, 0x57, 0xf3, 0xe6, 0x32, 0x0d, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x73, 0x74,
      0x61, 0x6d, 0x70, 0x00, 0x00, 0x00, 0x17, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01,
      0x00, 0x70, 0x00, 0x09, 0x00, 0x00, 0x00, 0x66, 0x72, 0x61, 0x6d, 0x65, 0x5f, 0x69, 0x64,
      0x00, 0x00, 0x00, 0xf2, 0xd4, 0x85, 0x4f, 0x13, 0xae, 0xf3, 0x2d, 0xfe, 0x21, 0x57, 0xf3,
      0xe6, 0x32, 0x0d, 0x00, 0x00, 0x72, 0x00, 0x00, 0x00, 0xf2, 0x51, 0x01, 0x00, 0x2d, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x25, 0x00, 0x00, 0x00, 0x62, 0x75, 0x69, 0x6c, 0x74,
      0x69, 0x6e, 0x5f, 0x69, 0x6e, 0x74, 0x65, 0x72, 0x66, 0x61, 0x63, 0x65, 0x73, 0x3a, 0x3a,
      0x6d, 0x73, 0x67, 0x3a, 0x3a, 0x64, 0x64, 0x73, 0x5f, 0x3a, 0x3a, 0x54, 0x69, 0x6d, 0x65,
      0x5f, 0x00, 0x00, 0x00, 0x00, 0x36, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x12, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x04, 0x00, 0x04, 0x00, 0x00, 0x00, 0x73,
      0x65, 0x63, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
      0x01, 0x00, 0x07, 0x00, 0x08, 0x00, 0x00, 0x00, 0x6e, 0x61, 0x6e, 0x6f, 0x73, 0x65, 0x63,
      0x00, 0x00, 0x00, 0xf2, 0xa3, 0xab, 0xc2, 0x35, 0xd1, 0x3d, 0xdb, 0xab, 0x7a, 0x36, 0x56,
      0x16, 0xf1, 0x08, 0x00, 0x00, 0x00, 0xa4, 0x00, 0x00, 0x00, 0xf2, 0x51, 0x01, 0x00, 0x2c,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00, 0x73, 0x65, 0x6e, 0x73,
      0x6f, 0x72, 0x5f, 0x6d, 0x73, 0x67, 0x73, 0x3a, 0x3a, 0x6d, 0x73, 0x67, 0x3a, 0x3a, 0x64,
      0x64, 0x73, 0x5f, 0x3a, 0x3a, 0x50, 0x6f, 0x69, 0x6e, 0x74, 0x46, 0x69, 0x65, 0x6c, 0x64,
      0x5f, 0x00, 0x6c, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x01, 0x00, 0x70, 0x00, 0x05, 0x00, 0x00, 0x00, 0x6e, 0x61, 0x6d, 0x65,
      0x00, 0x00, 0x00, 0x00, 0x15, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x07,
      0x00, 0x07, 0x00, 0x00, 0x00, 0x6f, 0x66, 0x66, 0x73, 0x65, 0x74, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x17, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x02, 0x00, 0x09,
      0x00, 0x00, 0x00, 0x64, 0x61, 0x74, 0x61, 0x74, 0x79, 0x70, 0x65, 0x00, 0x00, 0x00, 0x00,
      0x14, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x07, 0x00, 0x06, 0x00, 0x00,
      0x00, 0x63, 0x6f, 0x75, 0x6e, 0x74, 0x00, 0x00, 0x00, 0x7c, 0x00, 0x00, 0x00, 0x04, 0x00,
      0x00, 0x00, 0xf2, 0xf6, 0xf1, 0xa7, 0xb4, 0x13, 0x2f, 0x17, 0xb4, 0xae, 0x1d, 0x73, 0x77,
      0xc5, 0xbc, 0xf1, 0x7f, 0x62, 0xa4, 0x8e, 0xfc, 0x63, 0xa8, 0xa6, 0xfe, 0x5f, 0x9e, 0xda,
      0x06, 0xd7, 0xf2, 0xe5, 0x76, 0x5e, 0xc4, 0x8c, 0xff, 0xd4, 0x19, 0xed, 0x7f, 0xe8, 0x4e,
      0x2a, 0x55, 0xf1, 0xdc, 0xf1, 0x2c, 0xd2, 0xdd, 0x5e, 0x71, 0x2c, 0xb7, 0xb1, 0xe5, 0x1f,
      0xa3, 0xf2, 0xf2, 0xd4, 0x85, 0x4f, 0x13, 0xae, 0xf3, 0x2d, 0xfe, 0x21, 0x57, 0xf3, 0xe6,
      0x32, 0x0d, 0xf1, 0x56, 0x7c, 0x5a, 0x93, 0x54, 0x1c, 0x3b, 0x10, 0x86, 0xa4, 0xba, 0x46,
      0xf9, 0x8d, 0xf2, 0xa3, 0xab, 0xc2, 0x35, 0xd1, 0x3d, 0xdb, 0xab, 0x7a, 0x36, 0x56, 0x16,
      0xf1, 0x08, 0xf1, 0x82, 0x17, 0x10, 0x85, 0xb3, 0xf2, 0x74, 0x2b, 0xa8, 0x4e, 0xb5, 0xeb,
      0x0e, 0x0b,
  };
  return blob;
}
template <>
inline const uint8_t* TopicTraits<::sensor_msgs::msg::dds_::PointCloud2_>::type_info_blob() {
  static const uint8_t blob[] = {
      0xf0, 0x00, 0x00, 0x00, 0x01, 0x10, 0x00, 0x40, 0x70, 0x00, 0x00, 0x00, 0x6c, 0x00, 0x00,
      0x00, 0x14, 0x00, 0x00, 0x00, 0xf1, 0x7f, 0x62, 0xa4, 0x8e, 0xfc, 0x63, 0xa8, 0xa6, 0xfe,
      0x5f, 0x9e, 0xda, 0x06, 0xd7, 0x00, 0xcf, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x4c,
      0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0xf1, 0xdc, 0xf1, 0x2c,
      0xd2, 0xdd, 0x5e, 0x71, 0x2c, 0xb7, 0xb1, 0xe5, 0x1f, 0xa3, 0xf2, 0x00, 0x48, 0x00, 0x00,
      0x00, 0x14, 0x00, 0x00, 0x00, 0xf1, 0x56, 0x7c, 0x5a, 0x93, 0x54, 0x1c, 0x3b, 0x10, 0x86,
      0xa4, 0xba, 0x46, 0xf9, 0x8d, 0x00, 0x37, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0xf1,
      0x82, 0x17, 0x10, 0x85, 0xb3, 0xf2, 0x74, 0x2b, 0xa8, 0x4e, 0xb5, 0xeb, 0x0e, 0x0b, 0x00,
      0x57, 0x00, 0x00, 0x00, 0x02, 0x10, 0x00, 0x40, 0x70, 0x00, 0x00, 0x00, 0x6c, 0x00, 0x00,
      0x00, 0x14, 0x00, 0x00, 0x00, 0xf2, 0xf6, 0xf1, 0xa7, 0xb4, 0x13, 0x2f, 0x17, 0xb4, 0xae,
      0x1d, 0x73, 0x77, 0xc5, 0xbc, 0x00, 0x67, 0x01, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x4c,
      0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0xf2, 0xe5, 0x76, 0x5e,
      0xc4, 0x8c, 0xff, 0xd4, 0x19, 0xed, 0x7f, 0xe8, 0x4e, 0x2a, 0x55, 0x00, 0x7f, 0x00, 0x00,
      0x00, 0x14, 0x00, 0x00, 0x00, 0xf2, 0xd4, 0x85, 0x4f, 0x13, 0xae, 0xf3, 0x2d, 0xfe, 0x21,
      0x57, 0xf3, 0xe6, 0x32, 0x0d, 0x00, 0x76, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0xf2,
      0xa3, 0xab, 0xc2, 0x35, 0xd1, 0x3d, 0xdb, 0xab, 0x7a, 0x36, 0x56, 0x16, 0xf1, 0x08, 0x00,
      0xa8, 0x00, 0x00, 0x00,
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
struct topic_type_name<::sensor_msgs::msg::dds_::PointCloud2_> {
  static std::string value() {
    return org::eclipse::cyclonedds::topic::TopicTraits<
        ::sensor_msgs::msg::dds_::PointCloud2_>::getTypeName();
  }
};

}  // namespace topic
}  // namespace dds

REGISTER_TOPIC_TYPE(::sensor_msgs::msg::dds_::PointCloud2_)

namespace org {
namespace eclipse {
namespace cyclonedds {
namespace core {
namespace cdr {

template <>
propvec& get_type_props<::sensor_msgs::msg::dds_::PointCloud2_>();

template <typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true>
bool write(
    T& streamer,
    const ::sensor_msgs::msg::dds_::PointCloud2_& instance,
    entity_properties_t* props) {
  (void)instance;
  if (!streamer.start_struct(*props)) return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
        if (!streamer.start_member(*prop)) return false;
        if (!write(streamer, instance.header(), prop)) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 1:
        if (!streamer.start_member(*prop)) return false;
        if (!write(streamer, instance.height())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 2:
        if (!streamer.start_member(*prop)) return false;
        if (!write(streamer, instance.width())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 3:
        if (!streamer.start_member(*prop)) return false;
        if (!streamer.start_consecutive(false, false)) return false;
        {
          uint32_t se_1 = uint32_t(instance.fields().size());
          if (!write(streamer, se_1)) return false;
          for (uint32_t i_1 = 0; i_1 < se_1; i_1++) {
            if (!write(streamer, instance.fields()[i_1], prop)) return false;
          }  // i_1
        }    // end sequence 1
        if (!streamer.finish_consecutive()) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 4:
        if (!streamer.start_member(*prop)) return false;
        if (!write(streamer, instance.is_bigendian())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 5:
        if (!streamer.start_member(*prop)) return false;
        if (!write(streamer, instance.point_step())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 6:
        if (!streamer.start_member(*prop)) return false;
        if (!write(streamer, instance.row_step())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 7:
        if (!streamer.start_member(*prop)) return false;
        if (!streamer.start_consecutive(false, true)) return false;
        {
          uint32_t se_1 = uint32_t(instance.data().size());
          if (!write(streamer, se_1)) return false;
          if (se_1 > 0 && !write(streamer, instance.data()[0], se_1)) return false;
        }  // end sequence 1
        if (!streamer.finish_consecutive()) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 8:
        if (!streamer.start_member(*prop)) return false;
        if (!write(streamer, instance.is_dense())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template <typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true>
bool write(S& str, const ::sensor_msgs::msg::dds_::PointCloud2_& instance, bool as_key) {
  auto& props = get_type_props<::sensor_msgs::msg::dds_::PointCloud2_>();
  str.set_mode(cdr_stream::stream_mode::write, as_key);
  return write(str, instance, props.data());
}

template <typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true>
bool read(
    T& streamer, ::sensor_msgs::msg::dds_::PointCloud2_& instance, entity_properties_t* props) {
  (void)instance;
  if (!streamer.start_struct(*props)) return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
        if (!streamer.start_member(*prop)) return false;
        if (!read(streamer, instance.header(), prop)) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 1:
        if (!streamer.start_member(*prop)) return false;
        if (!read(streamer, instance.height())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 2:
        if (!streamer.start_member(*prop)) return false;
        if (!read(streamer, instance.width())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 3:
        if (!streamer.start_member(*prop)) return false;
        if (!streamer.start_consecutive(false, false)) return false;
        {
          uint32_t se_1 = uint32_t(instance.fields().size());
          if (!read(streamer, se_1)) return false;
          instance.fields().resize(se_1);
          for (uint32_t i_1 = 0; i_1 < se_1; i_1++) {
            if (!read(streamer, instance.fields()[i_1], prop)) return false;
          }  // i_1
        }    // end sequence 1
        if (!streamer.finish_consecutive()) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 4:
        if (!streamer.start_member(*prop)) return false;
        if (!read(streamer, instance.is_bigendian())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 5:
        if (!streamer.start_member(*prop)) return false;
        if (!read(streamer, instance.point_step())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 6:
        if (!streamer.start_member(*prop)) return false;
        if (!read(streamer, instance.row_step())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 7:
        if (!streamer.start_member(*prop)) return false;
        if (!streamer.start_consecutive(false, true)) return false;
        {
          uint32_t se_1 = uint32_t(instance.data().size());
          if (!read(streamer, se_1)) return false;
          instance.data().resize(se_1);
          if (se_1 > 0 && !read(streamer, instance.data()[0], se_1)) return false;
        }  // end sequence 1
        if (!streamer.finish_consecutive()) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 8:
        if (!streamer.start_member(*prop)) return false;
        if (!read(streamer, instance.is_dense())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template <typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true>
bool read(S& str, ::sensor_msgs::msg::dds_::PointCloud2_& instance, bool as_key) {
  auto& props = get_type_props<::sensor_msgs::msg::dds_::PointCloud2_>();
  str.set_mode(cdr_stream::stream_mode::read, as_key);
  return read(str, instance, props.data());
}

template <typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true>
bool move(
    T& streamer,
    const ::sensor_msgs::msg::dds_::PointCloud2_& instance,
    entity_properties_t* props) {
  (void)instance;
  if (!streamer.start_struct(*props)) return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
        if (!streamer.start_member(*prop)) return false;
        if (!move(streamer, instance.header(), prop)) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 1:
        if (!streamer.start_member(*prop)) return false;
        if (!move(streamer, instance.height())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 2:
        if (!streamer.start_member(*prop)) return false;
        if (!move(streamer, instance.width())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 3:
        if (!streamer.start_member(*prop)) return false;
        if (!streamer.start_consecutive(false, false)) return false;
        {
          uint32_t se_1 = uint32_t(instance.fields().size());
          if (!move(streamer, se_1)) return false;
          for (uint32_t i_1 = 0; i_1 < se_1; i_1++) {
            if (!move(streamer, instance.fields()[i_1], prop)) return false;
          }  // i_1
        }    // end sequence 1
        if (!streamer.finish_consecutive()) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 4:
        if (!streamer.start_member(*prop)) return false;
        if (!move(streamer, instance.is_bigendian())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 5:
        if (!streamer.start_member(*prop)) return false;
        if (!move(streamer, instance.point_step())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 6:
        if (!streamer.start_member(*prop)) return false;
        if (!move(streamer, instance.row_step())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 7:
        if (!streamer.start_member(*prop)) return false;
        if (!streamer.start_consecutive(false, true)) return false;
        {
          uint32_t se_1 = uint32_t(instance.data().size());
          if (!move(streamer, se_1)) return false;
          if (se_1 > 0 && !move(streamer, uint8_t(), se_1)) return false;
        }  // end sequence 1
        if (!streamer.finish_consecutive()) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 8:
        if (!streamer.start_member(*prop)) return false;
        if (!move(streamer, instance.is_dense())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template <typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true>
bool move(S& str, const ::sensor_msgs::msg::dds_::PointCloud2_& instance, bool as_key) {
  auto& props = get_type_props<::sensor_msgs::msg::dds_::PointCloud2_>();
  str.set_mode(cdr_stream::stream_mode::move, as_key);
  return move(str, instance, props.data());
}

template <typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true>
bool max(
    T& streamer,
    const ::sensor_msgs::msg::dds_::PointCloud2_& instance,
    entity_properties_t* props) {
  (void)instance;
  if (!streamer.start_struct(*props)) return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
        if (!streamer.start_member(*prop)) return false;
        if (!max(streamer, instance.header(), prop)) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 1:
        if (!streamer.start_member(*prop)) return false;
        if (!max(streamer, instance.height())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 2:
        if (!streamer.start_member(*prop)) return false;
        if (!max(streamer, instance.width())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 3:
        if (!streamer.start_member(*prop)) return false;
        if (!streamer.start_consecutive(false, false)) return false;
        {
          uint32_t se_1 = 0;
          if (!max(streamer, se_1)) return false;
          for (uint32_t i_1 = 0; i_1 < se_1; i_1++) {
            if (!max(streamer, instance.fields()[i_1], prop)) return false;
          }  // i_1
        }    // end sequence 1
        if (!streamer.finish_consecutive()) return false;
        streamer.position(SIZE_MAX);
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 4:
        if (!streamer.start_member(*prop)) return false;
        if (!max(streamer, instance.is_bigendian())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 5:
        if (!streamer.start_member(*prop)) return false;
        if (!max(streamer, instance.point_step())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 6:
        if (!streamer.start_member(*prop)) return false;
        if (!max(streamer, instance.row_step())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 7:
        if (!streamer.start_member(*prop)) return false;
        if (!streamer.start_consecutive(false, true)) return false;
        {
          uint32_t se_1 = 0;
          if (!max(streamer, se_1)) return false;
          if (se_1 > 0 && !max(streamer, uint8_t(), se_1)) return false;
        }  // end sequence 1
        if (!streamer.finish_consecutive()) return false;
        streamer.position(SIZE_MAX);
        if (!streamer.finish_member(*prop)) return false;
        break;
      case 8:
        if (!streamer.start_member(*prop)) return false;
        if (!max(streamer, instance.is_dense())) return false;
        if (!streamer.finish_member(*prop)) return false;
        break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template <typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true>
bool max(S& str, const ::sensor_msgs::msg::dds_::PointCloud2_& instance, bool as_key) {
  auto& props = get_type_props<::sensor_msgs::msg::dds_::PointCloud2_>();
  str.set_mode(cdr_stream::stream_mode::max, as_key);
  return max(str, instance, props.data());
}

}  // namespace cdr
}  // namespace core
}  // namespace cyclonedds
}  // namespace eclipse
}  // namespace org

#endif  // DDSCXX_POINTCLOUD2__HPP
