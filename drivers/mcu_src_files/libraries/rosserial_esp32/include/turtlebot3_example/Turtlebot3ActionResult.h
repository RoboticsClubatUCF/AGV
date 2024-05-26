#ifndef _ROS_turtlebot3_example_Turtlebot3ActionResult_h
#define _ROS_turtlebot3_example_Turtlebot3ActionResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "turtlebot3_example/Turtlebot3Result.h"

namespace turtlebot3_example
{

  class Turtlebot3ActionResult : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef turtlebot3_example::Turtlebot3Result _result_type;
      _result_type result;

    Turtlebot3ActionResult():
      header(),
      status(),
      result()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += this->result.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->result.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "turtlebot3_example/Turtlebot3ActionResult"; };
    virtual const char * getMD5() override { return "5489bcfa93e36e07a0b801f3ac4c7c97"; };

  };

}
#endif
