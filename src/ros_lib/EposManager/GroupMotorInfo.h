#ifndef _ROS_EposManager_GroupMotorInfo_h
#define _ROS_EposManager_GroupMotorInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "EposManager/MotorInfo.h"

namespace EposManager
{

  class GroupMotorInfo : public ros::Msg
  {
    public:
      uint8_t motor_group_length;
      EposManager::MotorInfo st_motor_group;
      EposManager::MotorInfo * motor_group;

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
        this->motor_group = (EposManager::MotorInfo*)realloc(this->motor_group, motor_group_lengthT * sizeof(EposManager::MotorInfo));
      offset += 3;
      motor_group_length = motor_group_lengthT;
      for( uint8_t i = 0; i < motor_group_length; i++){
      offset += this->st_motor_group.deserialize(inbuffer + offset);
        memcpy( &(this->motor_group[i]), &(this->st_motor_group), sizeof(EposManager::MotorInfo));
      }
     return offset;
    }

    const char * getType(){ return "EposManager/GroupMotorInfo"; };
    const char * getMD5(){ return "df4d06ffe0935e1bb3f70b2d4bb7a274"; };

  };

}
#endif