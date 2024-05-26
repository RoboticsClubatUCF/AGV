#ifndef _ROS_ainstein_radar_msgs_RadarTarget_h
#define _ROS_ainstein_radar_msgs_RadarTarget_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ainstein_radar_msgs
{

  class RadarTarget : public ros::Msg
  {
    public:
      typedef uint16_t _target_id_type;
      _target_id_type target_id;
      typedef double _snr_type;
      _snr_type snr;
      typedef double _range_type;
      _range_type range;
      typedef double _speed_type;
      _speed_type speed;
      typedef double _azimuth_type;
      _azimuth_type azimuth;
      typedef double _elevation_type;
      _elevation_type elevation;

    RadarTarget():
      target_id(0),
      snr(0),
      range(0),
      speed(0),
      azimuth(0),
      elevation(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->target_id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->target_id >> (8 * 1)) & 0xFF;
      offset += sizeof(this->target_id);
      union {
        double real;
        uint64_t base;
      } u_snr;
      u_snr.real = this->snr;
      *(outbuffer + offset + 0) = (u_snr.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_snr.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_snr.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_snr.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_snr.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_snr.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_snr.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_snr.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->snr);
      union {
        double real;
        uint64_t base;
      } u_range;
      u_range.real = this->range;
      *(outbuffer + offset + 0) = (u_range.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_range.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_range.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_range.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_range.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_range.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_range.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_range.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->range);
      union {
        double real;
        uint64_t base;
      } u_speed;
      u_speed.real = this->speed;
      *(outbuffer + offset + 0) = (u_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_speed.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_speed.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_speed.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_speed.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->speed);
      union {
        double real;
        uint64_t base;
      } u_azimuth;
      u_azimuth.real = this->azimuth;
      *(outbuffer + offset + 0) = (u_azimuth.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_azimuth.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_azimuth.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_azimuth.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_azimuth.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_azimuth.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_azimuth.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_azimuth.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->azimuth);
      union {
        double real;
        uint64_t base;
      } u_elevation;
      u_elevation.real = this->elevation;
      *(outbuffer + offset + 0) = (u_elevation.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_elevation.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_elevation.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_elevation.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_elevation.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_elevation.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_elevation.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_elevation.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->elevation);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->target_id =  ((uint16_t) (*(inbuffer + offset)));
      this->target_id |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->target_id);
      union {
        double real;
        uint64_t base;
      } u_snr;
      u_snr.base = 0;
      u_snr.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_snr.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_snr.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_snr.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_snr.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_snr.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_snr.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_snr.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->snr = u_snr.real;
      offset += sizeof(this->snr);
      union {
        double real;
        uint64_t base;
      } u_range;
      u_range.base = 0;
      u_range.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_range.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_range.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_range.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_range.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_range.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_range.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_range.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->range = u_range.real;
      offset += sizeof(this->range);
      union {
        double real;
        uint64_t base;
      } u_speed;
      u_speed.base = 0;
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->speed = u_speed.real;
      offset += sizeof(this->speed);
      union {
        double real;
        uint64_t base;
      } u_azimuth;
      u_azimuth.base = 0;
      u_azimuth.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_azimuth.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_azimuth.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_azimuth.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_azimuth.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_azimuth.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_azimuth.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_azimuth.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->azimuth = u_azimuth.real;
      offset += sizeof(this->azimuth);
      union {
        double real;
        uint64_t base;
      } u_elevation;
      u_elevation.base = 0;
      u_elevation.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_elevation.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_elevation.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_elevation.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_elevation.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_elevation.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_elevation.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_elevation.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->elevation = u_elevation.real;
      offset += sizeof(this->elevation);
     return offset;
    }

    virtual const char * getType() override { return "ainstein_radar_msgs/RadarTarget"; };
    virtual const char * getMD5() override { return "942d59f5ed2080c1747d5b79ce6dc1c0"; };

  };

}
#endif
