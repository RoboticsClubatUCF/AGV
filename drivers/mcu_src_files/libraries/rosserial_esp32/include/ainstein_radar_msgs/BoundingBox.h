#ifndef _ROS_ainstein_radar_msgs_BoundingBox_h
#define _ROS_ainstein_radar_msgs_BoundingBox_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"

namespace ainstein_radar_msgs
{

  class BoundingBox : public ros::Msg
  {
    public:
      typedef geometry_msgs::Pose _pose_type;
      _pose_type pose;
      typedef geometry_msgs::Vector3 _dimensions_type;
      _dimensions_type dimensions;
      typedef float _value_type;
      _value_type value;
      typedef uint32_t _label_type;
      _label_type label;

    BoundingBox():
      pose(),
      dimensions(),
      value(0),
      label(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->pose.serialize(outbuffer + offset);
      offset += this->dimensions.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_value;
      u_value.real = this->value;
      *(outbuffer + offset + 0) = (u_value.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_value.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_value.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_value.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->value);
      *(outbuffer + offset + 0) = (this->label >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->label >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->label >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->label >> (8 * 3)) & 0xFF;
      offset += sizeof(this->label);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->pose.deserialize(inbuffer + offset);
      offset += this->dimensions.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_value;
      u_value.base = 0;
      u_value.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_value.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_value.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_value.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->value = u_value.real;
      offset += sizeof(this->value);
      this->label =  ((uint32_t) (*(inbuffer + offset)));
      this->label |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->label |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->label |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->label);
     return offset;
    }

    virtual const char * getType() override { return "ainstein_radar_msgs/BoundingBox"; };
    virtual const char * getMD5() override { return "4214de46b5b5be0eae252609fbfa9f7a"; };

  };

}
#endif
