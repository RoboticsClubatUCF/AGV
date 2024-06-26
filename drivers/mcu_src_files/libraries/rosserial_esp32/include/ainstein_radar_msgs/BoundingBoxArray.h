#ifndef _ROS_ainstein_radar_msgs_BoundingBoxArray_h
#define _ROS_ainstein_radar_msgs_BoundingBoxArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "ainstein_radar_msgs/BoundingBox.h"

namespace ainstein_radar_msgs
{

  class BoundingBoxArray : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t boxes_length;
      typedef ainstein_radar_msgs::BoundingBox _boxes_type;
      _boxes_type st_boxes;
      _boxes_type * boxes;

    BoundingBoxArray():
      header(),
      boxes_length(0), st_boxes(), boxes(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->boxes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->boxes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->boxes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->boxes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->boxes_length);
      for( uint32_t i = 0; i < boxes_length; i++){
      offset += this->boxes[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t boxes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      boxes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      boxes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      boxes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->boxes_length);
      if(boxes_lengthT > boxes_length)
        this->boxes = (ainstein_radar_msgs::BoundingBox*)realloc(this->boxes, boxes_lengthT * sizeof(ainstein_radar_msgs::BoundingBox));
      boxes_length = boxes_lengthT;
      for( uint32_t i = 0; i < boxes_length; i++){
      offset += this->st_boxes.deserialize(inbuffer + offset);
        memcpy( &(this->boxes[i]), &(this->st_boxes), sizeof(ainstein_radar_msgs::BoundingBox));
      }
     return offset;
    }

    virtual const char * getType() override { return "ainstein_radar_msgs/BoundingBoxArray"; };
    virtual const char * getMD5() override { return "2ea1dfa56cda09dce8368482c68738e3"; };

  };

}
#endif
