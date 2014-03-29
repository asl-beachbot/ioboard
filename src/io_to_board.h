#ifndef IOMessages_to_board_h
#define IOMessages_to_board_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

class io_to_board : public ros::Msg
{
  public:
    uint16_t motor_right;
    uint16_t motor_left;
    uint8_t rake_flags;
    uint8_t status;

  virtual int serialize(unsigned char *outbuffer) const
  {
    int offset = 0;
    *(outbuffer + offset + 0) = (this->motor_right >> (8 * 0)) & 0xFF;
    *(outbuffer + offset + 1) = (this->motor_right >> (8 * 1)) & 0xFF;
    offset += sizeof(this->motor_right);
    *(outbuffer + offset + 0) = (this->motor_left >> (8 * 0)) & 0xFF;
    *(outbuffer + offset + 1) = (this->motor_left >> (8 * 1)) & 0xFF;
    offset += sizeof(this->motor_left);
    *(outbuffer + offset + 0) = (this->rake_flags >> (8 * 0)) & 0xFF;
    offset += sizeof(this->rake_flags);
    *(outbuffer + offset + 0) = (this->status >> (8 * 0)) & 0xFF;
    offset += sizeof(this->status);
    return offset;
  }

  virtual int deserialize(unsigned char *inbuffer)
  {
    int offset = 0;
    this->motor_right =  ((uint16_t) (*(inbuffer + offset)));
    this->motor_right |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
    offset += sizeof(this->motor_right);
    this->motor_left =  ((uint16_t) (*(inbuffer + offset)));
    this->motor_left |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
    offset += sizeof(this->motor_left);
    this->rake_flags =  ((uint8_t) (*(inbuffer + offset)));
    offset += sizeof(this->rake_flags);
    this->status =  ((uint8_t) (*(inbuffer + offset)));
    offset += sizeof(this->status);
    return offset;
  }

  const char * getType(){ return "rosserial_avr_tutorial/io_to_board"; };
  const char * getMD5(){ return "fd582db80d94d37c6883d648da94dd93"; };

};
#endif