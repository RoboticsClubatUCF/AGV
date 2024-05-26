#ifndef _ROS_ainstein_radar_msgs_RadarTrackedObjectStamped_h
#define _ROS_ainstein_radar_msgs_RadarTrackedObjectStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "ainstein_radar_msgs/RadarTrackedObject.h"

namespace ainstein_radar_msgs
{

  class RadarTrackedObjectStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef ainstein_radar_msgs::RadarTrackedObject _object_type;
      _object_type object;

    RadarTrackedObjectStamped():
      header(),
      object()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->object.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->object.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "ainstein_radar_msgs/RadarTrackedObjectStamped"; };
    virtual const char * getMD5() override { return "f602f4531922144f087b6812eed02f3e"; };

  };

}
#endif
