#ifndef _ROS_bbcontrol_Rake_h
#define _ROS_bbcontrol_Rake_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace bbcontrol
{

  class Rake : public ros::Msg
  {
    public:
      bool rakepin1;
      bool rakepin2;
      bool rakepin3;
      bool rakepin4;
      bool rakepin5;
      bool rakepin6;
      bool rakepin7;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_rakepin1;
      u_rakepin1.real = this->rakepin1;
      *(outbuffer + offset + 0) = (u_rakepin1.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->rakepin1);
      union {
        bool real;
        uint8_t base;
      } u_rakepin2;
      u_rakepin2.real = this->rakepin2;
      *(outbuffer + offset + 0) = (u_rakepin2.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->rakepin2);
      union {
        bool real;
        uint8_t base;
      } u_rakepin3;
      u_rakepin3.real = this->rakepin3;
      *(outbuffer + offset + 0) = (u_rakepin3.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->rakepin3);
      union {
        bool real;
        uint8_t base;
      } u_rakepin4;
      u_rakepin4.real = this->rakepin4;
      *(outbuffer + offset + 0) = (u_rakepin4.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->rakepin4);
      union {
        bool real;
        uint8_t base;
      } u_rakepin5;
      u_rakepin5.real = this->rakepin5;
      *(outbuffer + offset + 0) = (u_rakepin5.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->rakepin5);
      union {
        bool real;
        uint8_t base;
      } u_rakepin6;
      u_rakepin6.real = this->rakepin6;
      *(outbuffer + offset + 0) = (u_rakepin6.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->rakepin6);
      union {
        bool real;
        uint8_t base;
      } u_rakepin7;
      u_rakepin7.real = this->rakepin7;
      *(outbuffer + offset + 0) = (u_rakepin7.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->rakepin7);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_rakepin1;
      u_rakepin1.base = 0;
      u_rakepin1.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->rakepin1 = u_rakepin1.real;
      offset += sizeof(this->rakepin1);
      union {
        bool real;
        uint8_t base;
      } u_rakepin2;
      u_rakepin2.base = 0;
      u_rakepin2.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->rakepin2 = u_rakepin2.real;
      offset += sizeof(this->rakepin2);
      union {
        bool real;
        uint8_t base;
      } u_rakepin3;
      u_rakepin3.base = 0;
      u_rakepin3.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->rakepin3 = u_rakepin3.real;
      offset += sizeof(this->rakepin3);
      union {
        bool real;
        uint8_t base;
      } u_rakepin4;
      u_rakepin4.base = 0;
      u_rakepin4.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->rakepin4 = u_rakepin4.real;
      offset += sizeof(this->rakepin4);
      union {
        bool real;
        uint8_t base;
      } u_rakepin5;
      u_rakepin5.base = 0;
      u_rakepin5.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->rakepin5 = u_rakepin5.real;
      offset += sizeof(this->rakepin5);
      union {
        bool real;
        uint8_t base;
      } u_rakepin6;
      u_rakepin6.base = 0;
      u_rakepin6.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->rakepin6 = u_rakepin6.real;
      offset += sizeof(this->rakepin6);
      union {
        bool real;
        uint8_t base;
      } u_rakepin7;
      u_rakepin7.base = 0;
      u_rakepin7.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->rakepin7 = u_rakepin7.real;
      offset += sizeof(this->rakepin7);
     return offset;
    }

    const char * getType(){ return "bbcontrol/Rake"; };
    const char * getMD5(){ return "82127a08233496559b13c115b9aa298b"; };

  };

}
#endif