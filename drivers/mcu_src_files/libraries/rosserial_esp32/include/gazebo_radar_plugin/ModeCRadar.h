#ifndef _ROS_gazebo_radar_plugin_ModeCRadar_h
#define _ROS_gazebo_radar_plugin_ModeCRadar_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace gazebo_radar_plugin
{

  class ModeCRadar : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _range_type;
      _range_type range;
      typedef float _bearing_type;
      _bearing_type bearing;
      typedef float _altitude_type;
      _altitude_type altitude;
      typedef uint16_t _code_type;
      _code_type code;
      typedef bool _ident_type;
      _ident_type ident;

    ModeCRadar():
      header(),
      range(0),
      bearing(0),
      altitude(0),
      code(0),
      ident(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_range;
      u_range.real = this->range;
      *(outbuffer + offset + 0) = (u_range.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_range.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_range.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_range.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->range);
      union {
        float real;
        uint32_t base;
      } u_bearing;
      u_bearing.real = this->bearing;
      *(outbuffer + offset + 0) = (u_bearing.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bearing.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bearing.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bearing.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bearing);
      union {
        float real;
        uint32_t base;
      } u_altitude;
      u_altitude.real = this->altitude;
      *(outbuffer + offset + 0) = (u_altitude.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_altitude.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_altitude.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_altitude.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->altitude);
      *(outbuffer + offset + 0) = (this->code >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->code >> (8 * 1)) & 0xFF;
      offset += sizeof(this->code);
      union {
        bool real;
        uint8_t base;
      } u_ident;
      u_ident.real = this->ident;
      *(outbuffer + offset + 0) = (u_ident.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ident);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_range;
      u_range.base = 0;
      u_range.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_range.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_range.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_range.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->range = u_range.real;
      offset += sizeof(this->range);
      union {
        float real;
        uint32_t base;
      } u_bearing;
      u_bearing.base = 0;
      u_bearing.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bearing.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_bearing.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_bearing.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->bearing = u_bearing.real;
      offset += sizeof(this->bearing);
      union {
        float real;
        uint32_t base;
      } u_altitude;
      u_altitude.base = 0;
      u_altitude.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_altitude.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_altitude.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_altitude.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->altitude = u_altitude.real;
      offset += sizeof(this->altitude);
      this->code =  ((uint16_t) (*(inbuffer + offset)));
      this->code |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->code);
      union {
        bool real;
        uint8_t base;
      } u_ident;
      u_ident.base = 0;
      u_ident.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ident = u_ident.real;
      offset += sizeof(this->ident);
     return offset;
    }

    virtual const char * getType() override { return "gazebo_radar_plugin/ModeCRadar"; };
    virtual const char * getMD5() override { return "19c4f5ea27d7018d84f81236837f5bfe"; };

  };

}
#endif
