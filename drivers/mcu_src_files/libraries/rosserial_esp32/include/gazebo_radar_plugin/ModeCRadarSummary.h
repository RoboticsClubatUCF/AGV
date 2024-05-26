#ifndef _ROS_gazebo_radar_plugin_ModeCRadarSummary_h
#define _ROS_gazebo_radar_plugin_ModeCRadarSummary_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "gazebo_radar_plugin/ModeCRadar.h"

namespace gazebo_radar_plugin
{

  class ModeCRadarSummary : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t contacts_length;
      typedef gazebo_radar_plugin::ModeCRadar _contacts_type;
      _contacts_type st_contacts;
      _contacts_type * contacts;

    ModeCRadarSummary():
      header(),
      contacts_length(0), st_contacts(), contacts(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->contacts_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->contacts_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->contacts_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->contacts_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->contacts_length);
      for( uint32_t i = 0; i < contacts_length; i++){
      offset += this->contacts[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t contacts_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      contacts_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      contacts_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      contacts_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->contacts_length);
      if(contacts_lengthT > contacts_length)
        this->contacts = (gazebo_radar_plugin::ModeCRadar*)realloc(this->contacts, contacts_lengthT * sizeof(gazebo_radar_plugin::ModeCRadar));
      contacts_length = contacts_lengthT;
      for( uint32_t i = 0; i < contacts_length; i++){
      offset += this->st_contacts.deserialize(inbuffer + offset);
        memcpy( &(this->contacts[i]), &(this->st_contacts), sizeof(gazebo_radar_plugin::ModeCRadar));
      }
     return offset;
    }

    virtual const char * getType() override { return "gazebo_radar_plugin/ModeCRadarSummary"; };
    virtual const char * getMD5() override { return "98cae80bd5cdf50d70e731ef44eb76ea"; };

  };

}
#endif
