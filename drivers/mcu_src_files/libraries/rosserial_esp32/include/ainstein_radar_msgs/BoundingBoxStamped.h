#ifndef _ROS_ainstein_radar_msgs_BoundingBoxStamped_h
#define _ROS_ainstein_radar_msgs_BoundingBoxStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "ainstein_radar_msgs/BoundingBox.h"

namespace ainstein_radar_msgs
{

  class BoundingBoxStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef ainstein_radar_msgs::BoundingBox _box_type;
      _box_type box;

    BoundingBoxStamped():
      header(),
      box()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->box.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->box.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "ainstein_radar_msgs/BoundingBoxStamped"; };
    virtual const char * getMD5() override { return "259e334973915dc19ccfbff4a8ead75e"; };

  };

}
#endif
