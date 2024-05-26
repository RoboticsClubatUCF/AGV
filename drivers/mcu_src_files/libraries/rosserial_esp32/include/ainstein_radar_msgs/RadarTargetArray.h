#ifndef _ROS_ainstein_radar_msgs_RadarTargetArray_h
#define _ROS_ainstein_radar_msgs_RadarTargetArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "ainstein_radar_msgs/RadarTarget.h"

namespace ainstein_radar_msgs
{

  class RadarTargetArray : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t targets_length;
      typedef ainstein_radar_msgs::RadarTarget _targets_type;
      _targets_type st_targets;
      _targets_type * targets;

    RadarTargetArray():
      header(),
      targets_length(0), st_targets(), targets(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->targets_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->targets_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->targets_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->targets_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->targets_length);
      for( uint32_t i = 0; i < targets_length; i++){
      offset += this->targets[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t targets_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      targets_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      targets_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      targets_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->targets_length);
      if(targets_lengthT > targets_length)
        this->targets = (ainstein_radar_msgs::RadarTarget*)realloc(this->targets, targets_lengthT * sizeof(ainstein_radar_msgs::RadarTarget));
      targets_length = targets_lengthT;
      for( uint32_t i = 0; i < targets_length; i++){
      offset += this->st_targets.deserialize(inbuffer + offset);
        memcpy( &(this->targets[i]), &(this->st_targets), sizeof(ainstein_radar_msgs::RadarTarget));
      }
     return offset;
    }

    virtual const char * getType() override { return "ainstein_radar_msgs/RadarTargetArray"; };
    virtual const char * getMD5() override { return "e9ff18a480c0ec265f58b88134aeb493"; };

  };

}
#endif
