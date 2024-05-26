#ifndef _ROS_ainstein_radar_msgs_RadarTrackedObject_h
#define _ROS_ainstein_radar_msgs_RadarTrackedObject_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "ainstein_radar_msgs/BoundingBox.h"

namespace ainstein_radar_msgs
{

  class RadarTrackedObject : public ros::Msg
  {
    public:
      typedef uint16_t _id_type;
      _id_type id;
      typedef geometry_msgs::Pose _pose_type;
      _pose_type pose;
      typedef geometry_msgs::Twist _velocity_type;
      _velocity_type velocity;
      typedef ainstein_radar_msgs::BoundingBox _box_type;
      _box_type box;

    RadarTrackedObject():
      id(0),
      pose(),
      velocity(),
      box()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->id >> (8 * 1)) & 0xFF;
      offset += sizeof(this->id);
      offset += this->pose.serialize(outbuffer + offset);
      offset += this->velocity.serialize(outbuffer + offset);
      offset += this->box.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->id =  ((uint16_t) (*(inbuffer + offset)));
      this->id |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->id);
      offset += this->pose.deserialize(inbuffer + offset);
      offset += this->velocity.deserialize(inbuffer + offset);
      offset += this->box.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "ainstein_radar_msgs/RadarTrackedObject"; };
    virtual const char * getMD5() override { return "2a71462f8c844a31b18d0564b8ff5905"; };

  };

}
#endif
