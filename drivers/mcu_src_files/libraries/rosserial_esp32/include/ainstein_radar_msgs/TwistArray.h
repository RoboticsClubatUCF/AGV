#ifndef _ROS_ainstein_radar_msgs_TwistArray_h
#define _ROS_ainstein_radar_msgs_TwistArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Twist.h"

namespace ainstein_radar_msgs
{

  class TwistArray : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t velocities_length;
      typedef geometry_msgs::Twist _velocities_type;
      _velocities_type st_velocities;
      _velocities_type * velocities;

    TwistArray():
      header(),
      velocities_length(0), st_velocities(), velocities(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->velocities_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->velocities_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->velocities_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->velocities_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocities_length);
      for( uint32_t i = 0; i < velocities_length; i++){
      offset += this->velocities[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t velocities_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      velocities_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      velocities_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      velocities_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->velocities_length);
      if(velocities_lengthT > velocities_length)
        this->velocities = (geometry_msgs::Twist*)realloc(this->velocities, velocities_lengthT * sizeof(geometry_msgs::Twist));
      velocities_length = velocities_lengthT;
      for( uint32_t i = 0; i < velocities_length; i++){
      offset += this->st_velocities.deserialize(inbuffer + offset);
        memcpy( &(this->velocities[i]), &(this->st_velocities), sizeof(geometry_msgs::Twist));
      }
     return offset;
    }

    virtual const char * getType() override { return "ainstein_radar_msgs/TwistArray"; };
    virtual const char * getMD5() override { return "99d5a4212addb6ebc7a7ca0c92f09c0a"; };

  };

}
#endif
