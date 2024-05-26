#ifndef _ROS_ainstein_radar_msgs_RadarInfo_h
#define _ROS_ainstein_radar_msgs_RadarInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace ainstein_radar_msgs
{

  class RadarInfo : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef double _update_rate_type;
      _update_rate_type update_rate;
      typedef uint16_t _max_num_targets_type;
      _max_num_targets_type max_num_targets;
      typedef double _range_min_type;
      _range_min_type range_min;
      typedef double _range_max_type;
      _range_max_type range_max;
      typedef double _speed_min_type;
      _speed_min_type speed_min;
      typedef double _speed_max_type;
      _speed_max_type speed_max;
      typedef double _azimuth_min_type;
      _azimuth_min_type azimuth_min;
      typedef double _azimuth_max_type;
      _azimuth_max_type azimuth_max;
      typedef double _elevation_min_type;
      _elevation_min_type elevation_min;
      typedef double _elevation_max_type;
      _elevation_max_type elevation_max;
      typedef double _range_resolution_type;
      _range_resolution_type range_resolution;
      typedef double _range_accuracy_type;
      _range_accuracy_type range_accuracy;
      typedef double _speed_resolution_type;
      _speed_resolution_type speed_resolution;
      typedef double _speed_accuracy_type;
      _speed_accuracy_type speed_accuracy;
      typedef double _azimuth_resolution_type;
      _azimuth_resolution_type azimuth_resolution;
      typedef double _azimuth_accuracy_type;
      _azimuth_accuracy_type azimuth_accuracy;
      typedef double _elevation_resolution_type;
      _elevation_resolution_type elevation_resolution;
      typedef double _elevation_accuracy_type;
      _elevation_accuracy_type elevation_accuracy;

    RadarInfo():
      header(),
      update_rate(0),
      max_num_targets(0),
      range_min(0),
      range_max(0),
      speed_min(0),
      speed_max(0),
      azimuth_min(0),
      azimuth_max(0),
      elevation_min(0),
      elevation_max(0),
      range_resolution(0),
      range_accuracy(0),
      speed_resolution(0),
      speed_accuracy(0),
      azimuth_resolution(0),
      azimuth_accuracy(0),
      elevation_resolution(0),
      elevation_accuracy(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_update_rate;
      u_update_rate.real = this->update_rate;
      *(outbuffer + offset + 0) = (u_update_rate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_update_rate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_update_rate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_update_rate.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_update_rate.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_update_rate.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_update_rate.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_update_rate.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->update_rate);
      *(outbuffer + offset + 0) = (this->max_num_targets >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->max_num_targets >> (8 * 1)) & 0xFF;
      offset += sizeof(this->max_num_targets);
      union {
        double real;
        uint64_t base;
      } u_range_min;
      u_range_min.real = this->range_min;
      *(outbuffer + offset + 0) = (u_range_min.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_range_min.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_range_min.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_range_min.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_range_min.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_range_min.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_range_min.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_range_min.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->range_min);
      union {
        double real;
        uint64_t base;
      } u_range_max;
      u_range_max.real = this->range_max;
      *(outbuffer + offset + 0) = (u_range_max.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_range_max.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_range_max.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_range_max.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_range_max.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_range_max.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_range_max.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_range_max.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->range_max);
      union {
        double real;
        uint64_t base;
      } u_speed_min;
      u_speed_min.real = this->speed_min;
      *(outbuffer + offset + 0) = (u_speed_min.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_min.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed_min.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed_min.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_speed_min.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_speed_min.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_speed_min.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_speed_min.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->speed_min);
      union {
        double real;
        uint64_t base;
      } u_speed_max;
      u_speed_max.real = this->speed_max;
      *(outbuffer + offset + 0) = (u_speed_max.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_max.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed_max.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed_max.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_speed_max.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_speed_max.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_speed_max.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_speed_max.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->speed_max);
      union {
        double real;
        uint64_t base;
      } u_azimuth_min;
      u_azimuth_min.real = this->azimuth_min;
      *(outbuffer + offset + 0) = (u_azimuth_min.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_azimuth_min.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_azimuth_min.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_azimuth_min.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_azimuth_min.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_azimuth_min.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_azimuth_min.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_azimuth_min.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->azimuth_min);
      union {
        double real;
        uint64_t base;
      } u_azimuth_max;
      u_azimuth_max.real = this->azimuth_max;
      *(outbuffer + offset + 0) = (u_azimuth_max.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_azimuth_max.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_azimuth_max.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_azimuth_max.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_azimuth_max.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_azimuth_max.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_azimuth_max.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_azimuth_max.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->azimuth_max);
      union {
        double real;
        uint64_t base;
      } u_elevation_min;
      u_elevation_min.real = this->elevation_min;
      *(outbuffer + offset + 0) = (u_elevation_min.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_elevation_min.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_elevation_min.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_elevation_min.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_elevation_min.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_elevation_min.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_elevation_min.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_elevation_min.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->elevation_min);
      union {
        double real;
        uint64_t base;
      } u_elevation_max;
      u_elevation_max.real = this->elevation_max;
      *(outbuffer + offset + 0) = (u_elevation_max.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_elevation_max.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_elevation_max.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_elevation_max.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_elevation_max.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_elevation_max.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_elevation_max.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_elevation_max.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->elevation_max);
      union {
        double real;
        uint64_t base;
      } u_range_resolution;
      u_range_resolution.real = this->range_resolution;
      *(outbuffer + offset + 0) = (u_range_resolution.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_range_resolution.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_range_resolution.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_range_resolution.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_range_resolution.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_range_resolution.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_range_resolution.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_range_resolution.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->range_resolution);
      union {
        double real;
        uint64_t base;
      } u_range_accuracy;
      u_range_accuracy.real = this->range_accuracy;
      *(outbuffer + offset + 0) = (u_range_accuracy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_range_accuracy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_range_accuracy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_range_accuracy.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_range_accuracy.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_range_accuracy.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_range_accuracy.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_range_accuracy.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->range_accuracy);
      union {
        double real;
        uint64_t base;
      } u_speed_resolution;
      u_speed_resolution.real = this->speed_resolution;
      *(outbuffer + offset + 0) = (u_speed_resolution.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_resolution.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed_resolution.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed_resolution.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_speed_resolution.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_speed_resolution.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_speed_resolution.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_speed_resolution.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->speed_resolution);
      union {
        double real;
        uint64_t base;
      } u_speed_accuracy;
      u_speed_accuracy.real = this->speed_accuracy;
      *(outbuffer + offset + 0) = (u_speed_accuracy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_accuracy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed_accuracy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed_accuracy.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_speed_accuracy.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_speed_accuracy.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_speed_accuracy.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_speed_accuracy.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->speed_accuracy);
      union {
        double real;
        uint64_t base;
      } u_azimuth_resolution;
      u_azimuth_resolution.real = this->azimuth_resolution;
      *(outbuffer + offset + 0) = (u_azimuth_resolution.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_azimuth_resolution.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_azimuth_resolution.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_azimuth_resolution.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_azimuth_resolution.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_azimuth_resolution.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_azimuth_resolution.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_azimuth_resolution.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->azimuth_resolution);
      union {
        double real;
        uint64_t base;
      } u_azimuth_accuracy;
      u_azimuth_accuracy.real = this->azimuth_accuracy;
      *(outbuffer + offset + 0) = (u_azimuth_accuracy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_azimuth_accuracy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_azimuth_accuracy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_azimuth_accuracy.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_azimuth_accuracy.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_azimuth_accuracy.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_azimuth_accuracy.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_azimuth_accuracy.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->azimuth_accuracy);
      union {
        double real;
        uint64_t base;
      } u_elevation_resolution;
      u_elevation_resolution.real = this->elevation_resolution;
      *(outbuffer + offset + 0) = (u_elevation_resolution.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_elevation_resolution.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_elevation_resolution.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_elevation_resolution.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_elevation_resolution.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_elevation_resolution.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_elevation_resolution.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_elevation_resolution.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->elevation_resolution);
      union {
        double real;
        uint64_t base;
      } u_elevation_accuracy;
      u_elevation_accuracy.real = this->elevation_accuracy;
      *(outbuffer + offset + 0) = (u_elevation_accuracy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_elevation_accuracy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_elevation_accuracy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_elevation_accuracy.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_elevation_accuracy.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_elevation_accuracy.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_elevation_accuracy.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_elevation_accuracy.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->elevation_accuracy);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_update_rate;
      u_update_rate.base = 0;
      u_update_rate.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_update_rate.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_update_rate.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_update_rate.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_update_rate.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_update_rate.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_update_rate.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_update_rate.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->update_rate = u_update_rate.real;
      offset += sizeof(this->update_rate);
      this->max_num_targets =  ((uint16_t) (*(inbuffer + offset)));
      this->max_num_targets |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->max_num_targets);
      union {
        double real;
        uint64_t base;
      } u_range_min;
      u_range_min.base = 0;
      u_range_min.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_range_min.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_range_min.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_range_min.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_range_min.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_range_min.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_range_min.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_range_min.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->range_min = u_range_min.real;
      offset += sizeof(this->range_min);
      union {
        double real;
        uint64_t base;
      } u_range_max;
      u_range_max.base = 0;
      u_range_max.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_range_max.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_range_max.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_range_max.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_range_max.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_range_max.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_range_max.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_range_max.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->range_max = u_range_max.real;
      offset += sizeof(this->range_max);
      union {
        double real;
        uint64_t base;
      } u_speed_min;
      u_speed_min.base = 0;
      u_speed_min.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_min.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed_min.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed_min.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_speed_min.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_speed_min.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_speed_min.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_speed_min.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->speed_min = u_speed_min.real;
      offset += sizeof(this->speed_min);
      union {
        double real;
        uint64_t base;
      } u_speed_max;
      u_speed_max.base = 0;
      u_speed_max.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_max.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed_max.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed_max.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_speed_max.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_speed_max.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_speed_max.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_speed_max.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->speed_max = u_speed_max.real;
      offset += sizeof(this->speed_max);
      union {
        double real;
        uint64_t base;
      } u_azimuth_min;
      u_azimuth_min.base = 0;
      u_azimuth_min.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_azimuth_min.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_azimuth_min.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_azimuth_min.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_azimuth_min.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_azimuth_min.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_azimuth_min.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_azimuth_min.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->azimuth_min = u_azimuth_min.real;
      offset += sizeof(this->azimuth_min);
      union {
        double real;
        uint64_t base;
      } u_azimuth_max;
      u_azimuth_max.base = 0;
      u_azimuth_max.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_azimuth_max.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_azimuth_max.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_azimuth_max.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_azimuth_max.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_azimuth_max.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_azimuth_max.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_azimuth_max.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->azimuth_max = u_azimuth_max.real;
      offset += sizeof(this->azimuth_max);
      union {
        double real;
        uint64_t base;
      } u_elevation_min;
      u_elevation_min.base = 0;
      u_elevation_min.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_elevation_min.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_elevation_min.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_elevation_min.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_elevation_min.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_elevation_min.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_elevation_min.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_elevation_min.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->elevation_min = u_elevation_min.real;
      offset += sizeof(this->elevation_min);
      union {
        double real;
        uint64_t base;
      } u_elevation_max;
      u_elevation_max.base = 0;
      u_elevation_max.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_elevation_max.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_elevation_max.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_elevation_max.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_elevation_max.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_elevation_max.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_elevation_max.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_elevation_max.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->elevation_max = u_elevation_max.real;
      offset += sizeof(this->elevation_max);
      union {
        double real;
        uint64_t base;
      } u_range_resolution;
      u_range_resolution.base = 0;
      u_range_resolution.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_range_resolution.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_range_resolution.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_range_resolution.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_range_resolution.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_range_resolution.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_range_resolution.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_range_resolution.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->range_resolution = u_range_resolution.real;
      offset += sizeof(this->range_resolution);
      union {
        double real;
        uint64_t base;
      } u_range_accuracy;
      u_range_accuracy.base = 0;
      u_range_accuracy.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_range_accuracy.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_range_accuracy.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_range_accuracy.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_range_accuracy.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_range_accuracy.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_range_accuracy.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_range_accuracy.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->range_accuracy = u_range_accuracy.real;
      offset += sizeof(this->range_accuracy);
      union {
        double real;
        uint64_t base;
      } u_speed_resolution;
      u_speed_resolution.base = 0;
      u_speed_resolution.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_resolution.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed_resolution.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed_resolution.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_speed_resolution.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_speed_resolution.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_speed_resolution.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_speed_resolution.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->speed_resolution = u_speed_resolution.real;
      offset += sizeof(this->speed_resolution);
      union {
        double real;
        uint64_t base;
      } u_speed_accuracy;
      u_speed_accuracy.base = 0;
      u_speed_accuracy.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_accuracy.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed_accuracy.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed_accuracy.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_speed_accuracy.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_speed_accuracy.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_speed_accuracy.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_speed_accuracy.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->speed_accuracy = u_speed_accuracy.real;
      offset += sizeof(this->speed_accuracy);
      union {
        double real;
        uint64_t base;
      } u_azimuth_resolution;
      u_azimuth_resolution.base = 0;
      u_azimuth_resolution.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_azimuth_resolution.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_azimuth_resolution.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_azimuth_resolution.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_azimuth_resolution.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_azimuth_resolution.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_azimuth_resolution.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_azimuth_resolution.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->azimuth_resolution = u_azimuth_resolution.real;
      offset += sizeof(this->azimuth_resolution);
      union {
        double real;
        uint64_t base;
      } u_azimuth_accuracy;
      u_azimuth_accuracy.base = 0;
      u_azimuth_accuracy.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_azimuth_accuracy.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_azimuth_accuracy.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_azimuth_accuracy.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_azimuth_accuracy.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_azimuth_accuracy.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_azimuth_accuracy.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_azimuth_accuracy.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->azimuth_accuracy = u_azimuth_accuracy.real;
      offset += sizeof(this->azimuth_accuracy);
      union {
        double real;
        uint64_t base;
      } u_elevation_resolution;
      u_elevation_resolution.base = 0;
      u_elevation_resolution.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_elevation_resolution.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_elevation_resolution.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_elevation_resolution.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_elevation_resolution.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_elevation_resolution.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_elevation_resolution.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_elevation_resolution.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->elevation_resolution = u_elevation_resolution.real;
      offset += sizeof(this->elevation_resolution);
      union {
        double real;
        uint64_t base;
      } u_elevation_accuracy;
      u_elevation_accuracy.base = 0;
      u_elevation_accuracy.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_elevation_accuracy.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_elevation_accuracy.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_elevation_accuracy.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_elevation_accuracy.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_elevation_accuracy.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_elevation_accuracy.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_elevation_accuracy.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->elevation_accuracy = u_elevation_accuracy.real;
      offset += sizeof(this->elevation_accuracy);
     return offset;
    }

    virtual const char * getType() override { return "ainstein_radar_msgs/RadarInfo"; };
    virtual const char * getMD5() override { return "517f7ed5e498f66b6af3caaa2c6f2059"; };

  };

}
#endif
