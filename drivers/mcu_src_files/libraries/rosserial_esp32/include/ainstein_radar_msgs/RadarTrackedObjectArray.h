#ifndef _ROS_ainstein_radar_msgs_RadarTrackedObjectArray_h
#define _ROS_ainstein_radar_msgs_RadarTrackedObjectArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "ainstein_radar_msgs/RadarTrackedObject.h"

namespace ainstein_radar_msgs
{

  class RadarTrackedObjectArray : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t objects_length;
      typedef ainstein_radar_msgs::RadarTrackedObject _objects_type;
      _objects_type st_objects;
      _objects_type * objects;

    RadarTrackedObjectArray():
      header(),
      objects_length(0), st_objects(), objects(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->objects_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->objects_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->objects_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->objects_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->objects_length);
      for( uint32_t i = 0; i < objects_length; i++){
      offset += this->objects[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t objects_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      objects_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      objects_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      objects_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->objects_length);
      if(objects_lengthT > objects_length)
        this->objects = (ainstein_radar_msgs::RadarTrackedObject*)realloc(this->objects, objects_lengthT * sizeof(ainstein_radar_msgs::RadarTrackedObject));
      objects_length = objects_lengthT;
      for( uint32_t i = 0; i < objects_length; i++){
      offset += this->st_objects.deserialize(inbuffer + offset);
        memcpy( &(this->objects[i]), &(this->st_objects), sizeof(ainstein_radar_msgs::RadarTrackedObject));
      }
     return offset;
    }

    virtual const char * getType() override { return "ainstein_radar_msgs/RadarTrackedObjectArray"; };
    virtual const char * getMD5() override { return "599beb468e0d0aa0f7e1d3a5ae39243b"; };

  };

}
#endif
