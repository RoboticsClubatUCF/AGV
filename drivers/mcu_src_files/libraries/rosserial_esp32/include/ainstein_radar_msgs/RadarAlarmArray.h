#ifndef _ROS_ainstein_radar_msgs_RadarAlarmArray_h
#define _ROS_ainstein_radar_msgs_RadarAlarmArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "ainstein_radar_msgs/RadarAlarm.h"

namespace ainstein_radar_msgs
{

  class RadarAlarmArray : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t alarms_length;
      typedef ainstein_radar_msgs::RadarAlarm _alarms_type;
      _alarms_type st_alarms;
      _alarms_type * alarms;

    RadarAlarmArray():
      header(),
      alarms_length(0), st_alarms(), alarms(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->alarms_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->alarms_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->alarms_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->alarms_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->alarms_length);
      for( uint32_t i = 0; i < alarms_length; i++){
      offset += this->alarms[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t alarms_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      alarms_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      alarms_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      alarms_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->alarms_length);
      if(alarms_lengthT > alarms_length)
        this->alarms = (ainstein_radar_msgs::RadarAlarm*)realloc(this->alarms, alarms_lengthT * sizeof(ainstein_radar_msgs::RadarAlarm));
      alarms_length = alarms_lengthT;
      for( uint32_t i = 0; i < alarms_length; i++){
      offset += this->st_alarms.deserialize(inbuffer + offset);
        memcpy( &(this->alarms[i]), &(this->st_alarms), sizeof(ainstein_radar_msgs::RadarAlarm));
      }
     return offset;
    }

    virtual const char * getType() override { return "ainstein_radar_msgs/RadarAlarmArray"; };
    virtual const char * getMD5() override { return "a2ed2b28cdaf4bd421182c41d9016e57"; };

  };

}
#endif
