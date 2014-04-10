#ifndef _ROS_EposManager_MotorInfo_h
#define _ROS_EposManager_MotorInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace EposManager
{

  class MotorInfo : public ros::Msg
  {
    public:
      uint16_t node_id;
      char * motor_name;
      uint16_t state;
      char * faults;
      int32_t motor_velocity;
      int32_t motor_position;
      int32_t motor_current;
      ros::Time stamp;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->node_id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->node_id >> (8 * 1)) & 0xFF;
      offset += sizeof(this->node_id);
      uint32_t length_motor_name = strlen( (const char*) this->motor_name);
      memcpy(outbuffer + offset, &length_motor_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->motor_name, length_motor_name);
      offset += length_motor_name;
      *(outbuffer + offset + 0) = (this->state >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->state >> (8 * 1)) & 0xFF;
      offset += sizeof(this->state);
      uint32_t length_faults = strlen( (const char*) this->faults);
      memcpy(outbuffer + offset, &length_faults, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->faults, length_faults);
      offset += length_faults;
      union {
        int32_t real;
        uint32_t base;
      } u_motor_velocity;
      u_motor_velocity.real = this->motor_velocity;
      *(outbuffer + offset + 0) = (u_motor_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motor_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motor_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor_velocity);
      union {
        int32_t real;
        uint32_t base;
      } u_motor_position;
      u_motor_position.real = this->motor_position;
      *(outbuffer + offset + 0) = (u_motor_position.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor_position.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motor_position.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motor_position.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor_position);
      union {
        int32_t real;
        uint32_t base;
      } u_motor_current;
      u_motor_current.real = this->motor_current;
      *(outbuffer + offset + 0) = (u_motor_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motor_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motor_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor_current);
      *(outbuffer + offset + 0) = (this->stamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.sec);
      *(outbuffer + offset + 0) = (this->stamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.nsec);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->node_id =  ((uint16_t) (*(inbuffer + offset)));
      this->node_id |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->node_id);
      uint32_t length_motor_name;
      memcpy(&length_motor_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_motor_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_motor_name-1]=0;
      this->motor_name = (char *)(inbuffer + offset-1);
      offset += length_motor_name;
      this->state =  ((uint16_t) (*(inbuffer + offset)));
      this->state |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->state);
      uint32_t length_faults;
      memcpy(&length_faults, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_faults; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_faults-1]=0;
      this->faults = (char *)(inbuffer + offset-1);
      offset += length_faults;
      union {
        int32_t real;
        uint32_t base;
      } u_motor_velocity;
      u_motor_velocity.base = 0;
      u_motor_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_motor_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_motor_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->motor_velocity = u_motor_velocity.real;
      offset += sizeof(this->motor_velocity);
      union {
        int32_t real;
        uint32_t base;
      } u_motor_position;
      u_motor_position.base = 0;
      u_motor_position.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor_position.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_motor_position.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_motor_position.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->motor_position = u_motor_position.real;
      offset += sizeof(this->motor_position);
      union {
        int32_t real;
        uint32_t base;
      } u_motor_current;
      u_motor_current.base = 0;
      u_motor_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_motor_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_motor_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->motor_current = u_motor_current.real;
      offset += sizeof(this->motor_current);
      this->stamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.sec);
      this->stamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.nsec);
     return offset;
    }

    const char * getType(){ return "EposManager/MotorInfo"; };
    const char * getMD5(){ return "f0af010a942d4953f1738bc4cb3bf83c"; };

  };

}
#endif