#ifndef _ROS_ugv_msg_RC_h
#define _ROS_ugv_msg_RC_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace ugv_msg
{

  class RC : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int16_t _right_y_type;
      _right_y_type right_y;
      typedef int16_t _right_x_type;
      _right_x_type right_x;
      typedef int16_t _left_y_type;
      _left_y_type left_y;
      typedef int16_t _left_x_type;
      _left_x_type left_x;
      typedef bool _switch_a_type;
      _switch_a_type switch_a;
      typedef bool _switch_b_type;
      _switch_b_type switch_b;
      typedef bool _switch_d_type;
      _switch_d_type switch_d;
      typedef bool _switch_e_type;
      _switch_e_type switch_e;
      typedef bool _switch_f_type;
      _switch_f_type switch_f;
      typedef int16_t _switch_g_type;
      _switch_g_type switch_g;

    RC():
      header(),
      right_y(0),
      right_x(0),
      left_y(0),
      left_x(0),
      switch_a(0),
      switch_b(0),
      switch_d(0),
      switch_e(0),
      switch_f(0),
      switch_g(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int16_t real;
        uint16_t base;
      } u_right_y;
      u_right_y.real = this->right_y;
      *(outbuffer + offset + 0) = (u_right_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_y.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->right_y);
      union {
        int16_t real;
        uint16_t base;
      } u_right_x;
      u_right_x.real = this->right_x;
      *(outbuffer + offset + 0) = (u_right_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_x.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->right_x);
      union {
        int16_t real;
        uint16_t base;
      } u_left_y;
      u_left_y.real = this->left_y;
      *(outbuffer + offset + 0) = (u_left_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_y.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->left_y);
      union {
        int16_t real;
        uint16_t base;
      } u_left_x;
      u_left_x.real = this->left_x;
      *(outbuffer + offset + 0) = (u_left_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_x.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->left_x);
      union {
        bool real;
        uint8_t base;
      } u_switch_a;
      u_switch_a.real = this->switch_a;
      *(outbuffer + offset + 0) = (u_switch_a.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->switch_a);
      union {
        bool real;
        uint8_t base;
      } u_switch_b;
      u_switch_b.real = this->switch_b;
      *(outbuffer + offset + 0) = (u_switch_b.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->switch_b);
      union {
        bool real;
        uint8_t base;
      } u_switch_d;
      u_switch_d.real = this->switch_d;
      *(outbuffer + offset + 0) = (u_switch_d.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->switch_d);
      union {
        bool real;
        uint8_t base;
      } u_switch_e;
      u_switch_e.real = this->switch_e;
      *(outbuffer + offset + 0) = (u_switch_e.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->switch_e);
      union {
        bool real;
        uint8_t base;
      } u_switch_f;
      u_switch_f.real = this->switch_f;
      *(outbuffer + offset + 0) = (u_switch_f.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->switch_f);
      union {
        int16_t real;
        uint16_t base;
      } u_switch_g;
      u_switch_g.real = this->switch_g;
      *(outbuffer + offset + 0) = (u_switch_g.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_switch_g.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->switch_g);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int16_t real;
        uint16_t base;
      } u_right_y;
      u_right_y.base = 0;
      u_right_y.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_y.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->right_y = u_right_y.real;
      offset += sizeof(this->right_y);
      union {
        int16_t real;
        uint16_t base;
      } u_right_x;
      u_right_x.base = 0;
      u_right_x.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_x.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->right_x = u_right_x.real;
      offset += sizeof(this->right_x);
      union {
        int16_t real;
        uint16_t base;
      } u_left_y;
      u_left_y.base = 0;
      u_left_y.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_y.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->left_y = u_left_y.real;
      offset += sizeof(this->left_y);
      union {
        int16_t real;
        uint16_t base;
      } u_left_x;
      u_left_x.base = 0;
      u_left_x.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_x.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->left_x = u_left_x.real;
      offset += sizeof(this->left_x);
      union {
        bool real;
        uint8_t base;
      } u_switch_a;
      u_switch_a.base = 0;
      u_switch_a.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->switch_a = u_switch_a.real;
      offset += sizeof(this->switch_a);
      union {
        bool real;
        uint8_t base;
      } u_switch_b;
      u_switch_b.base = 0;
      u_switch_b.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->switch_b = u_switch_b.real;
      offset += sizeof(this->switch_b);
      union {
        bool real;
        uint8_t base;
      } u_switch_d;
      u_switch_d.base = 0;
      u_switch_d.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->switch_d = u_switch_d.real;
      offset += sizeof(this->switch_d);
      union {
        bool real;
        uint8_t base;
      } u_switch_e;
      u_switch_e.base = 0;
      u_switch_e.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->switch_e = u_switch_e.real;
      offset += sizeof(this->switch_e);
      union {
        bool real;
        uint8_t base;
      } u_switch_f;
      u_switch_f.base = 0;
      u_switch_f.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->switch_f = u_switch_f.real;
      offset += sizeof(this->switch_f);
      union {
        int16_t real;
        uint16_t base;
      } u_switch_g;
      u_switch_g.base = 0;
      u_switch_g.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_switch_g.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->switch_g = u_switch_g.real;
      offset += sizeof(this->switch_g);
     return offset;
    }

    virtual const char * getType() override { return "ugv_msg/RC"; };
    virtual const char * getMD5() override { return "ae5cd314761928c2bbf21b70918a3450"; };

  };

}
#endif
