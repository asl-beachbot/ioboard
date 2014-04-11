#ifndef _ROS_EposManager_GroupEPOSControl_h
#define _ROS_EposManager_GroupEPOSControl_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "EposManager/EPOSControl.h"

namespace EposManager
{

  class GroupEPOSControl : public ros::Msg
  {
    public:
      uint8_t motor_group_length;
      EposManager::EPOSControl st_motor_group;
      EposManager::EPOSControl * motor_group;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = motor_group_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < motor_group_length; i++){
      offset += this->motor_group[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t motor_group_lengthT = *(inbuffer + offset++);
      if(motor_group_lengthT > motor_group_length)
        this->motor_group = (EposManager::EPOSControl*)realloc(this->motor_group, motor_group_lengthT * sizeof(EposManager::EPOSControl));
      offset += 3;
      motor_group_length = motor_group_lengthT;
      for( uint8_t i = 0; i < motor_group_length; i++){
      offset += this->st_motor_group.deserialize(inbuffer + offset);
        memcpy( &(this->motor_group[i]), &(this->st_motor_group), sizeof(EposManager::EPOSControl));
      }
     return offset;
    }

    const char * getType(){ return "EposManager/GroupEPOSControl"; };
    const char * getMD5(){ return "907295e8eaf96445684ec0fc29dcf40d"; };

  };

}
#endif