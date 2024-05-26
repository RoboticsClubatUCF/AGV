#ifndef _ROS_ainstein_radar_msgs_RadarTargetStamped_h
#define _ROS_ainstein_radar_msgs_RadarTargetStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "ainstein_radar_msgs/RadarTarget.h"

namespace ainstein_radar_msgs
{

  class RadarTargetStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef ainstein_radar_msgs::RadarTarget _target_type;
      _target_type target;

    RadarTargetStamped():
      header(),
      target()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->target.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->target.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "ainstein_radar_msgs/RadarTargetStamped"; };
    virtual const char * getMD5() override { return "b1fbe9d124fcb7889ff4a0fa4df5665a"; };

  };

}
#endif
