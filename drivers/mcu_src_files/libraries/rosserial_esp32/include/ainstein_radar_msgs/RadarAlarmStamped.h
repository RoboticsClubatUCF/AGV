#ifndef _ROS_ainstein_radar_msgs_RadarAlarmStamped_h
#define _ROS_ainstein_radar_msgs_RadarAlarmStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace ainstein_radar_msgs
{

  class RadarAlarmStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef bool _LCA_alarm_type;
      _LCA_alarm_type LCA_alarm;
      typedef bool _CVW_alarm_type;
      _CVW_alarm_type CVW_alarm;
      typedef bool _BSD_alarm_type;
      _BSD_alarm_type BSD_alarm;

    RadarAlarmStamped():
      header(),
      LCA_alarm(0),
      CVW_alarm(0),
      BSD_alarm(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_LCA_alarm;
      u_LCA_alarm.real = this->LCA_alarm;
      *(outbuffer + offset + 0) = (u_LCA_alarm.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->LCA_alarm);
      union {
        bool real;
        uint8_t base;
      } u_CVW_alarm;
      u_CVW_alarm.real = this->CVW_alarm;
      *(outbuffer + offset + 0) = (u_CVW_alarm.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->CVW_alarm);
      union {
        bool real;
        uint8_t base;
      } u_BSD_alarm;
      u_BSD_alarm.real = this->BSD_alarm;
      *(outbuffer + offset + 0) = (u_BSD_alarm.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->BSD_alarm);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_LCA_alarm;
      u_LCA_alarm.base = 0;
      u_LCA_alarm.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->LCA_alarm = u_LCA_alarm.real;
      offset += sizeof(this->LCA_alarm);
      union {
        bool real;
        uint8_t base;
      } u_CVW_alarm;
      u_CVW_alarm.base = 0;
      u_CVW_alarm.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->CVW_alarm = u_CVW_alarm.real;
      offset += sizeof(this->CVW_alarm);
      union {
        bool real;
        uint8_t base;
      } u_BSD_alarm;
      u_BSD_alarm.base = 0;
      u_BSD_alarm.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->BSD_alarm = u_BSD_alarm.real;
      offset += sizeof(this->BSD_alarm);
     return offset;
    }

    virtual const char * getType() override { return "ainstein_radar_msgs/RadarAlarmStamped"; };
    virtual const char * getMD5() override { return "c8302a7df5f42db7b8e1cc9a4e4b392a"; };

  };

}
#endif
