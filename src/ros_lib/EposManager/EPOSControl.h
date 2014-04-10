#ifndef _ROS_EposManager_EPOSControl_h
#define _ROS_EposManager_EPOSControl_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace EposManager
{

  class EPOSControl : public ros::Msg
  {
    public:
      uint16_t node_id;
      uint8_t control_mode;
      int32_t setpoint;
      enum { VELOCITY =  1 };
      enum { ABSOLUTE_POSITION =  2 };
      enum { ABSOLUTE_POSITION_IMMEDIATE =  3 };
      enum { RELATIVE_POSITION =  4 };
      enum { RELATIVE_POSITION_IMMEDIATE =  5 };
      enum { HOMING =  6 };

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->node_id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->node_id >> (8 * 1)) & 0xFF;
      offset += sizeof(this->node_id);
      *(outbuffer + offset + 0) = (this->control_mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->control_mode);
      union {
        int32_t real;
        uint32_t base;
      } u_setpoint;
      u_setpoint.real = this->setpoint;
      *(outbuffer + offset + 0) = (u_setpoint.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_setpoint.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_setpoint.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_setpoint.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->setpoint);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->node_id =  ((uint16_t) (*(inbuffer + offset)));
      this->node_id |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->node_id);
      this->control_mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->control_mode);
      union {
        int32_t real;
        uint32_t base;
      } u_setpoint;
      u_setpoint.base = 0;
      u_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->setpoint = u_setpoint.real;
      offset += sizeof(this->setpoint);
     return offset;
    }

    const char * getType(){ return "EposManager/EPOSControl"; };
    const char * getMD5(){ return "d72128e97db1f4de4932a96f7baa7c0f"; };

  };

}
#endif