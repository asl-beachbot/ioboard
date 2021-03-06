#ifndef IOMessages_to_board_h
#define IOMessages_to_board_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

class io_to_board : public ros::Msg
{
  public:
  uint16_t motor_left;
  uint16_t motor_right;
  uint8_t rake_flags;
  uint8_t status_motors;
  uint8_t status_charger;
  uint8_t status_additions;

  virtual int serialize(unsigned char *outbuffer) const
  {
    int offset = 0;

    *(outbuffer + offset + 0) = (this->motor_left >> (8 * 0)) & 0xFF;
    *(outbuffer + offset + 1) = (this->motor_left >> (8 * 1)) & 0xFF;
    offset += sizeof(this->motor_left);

    *(outbuffer + offset + 0) = (this->motor_right >> (8 * 0)) & 0xFF;
    *(outbuffer + offset + 1) = (this->motor_right >> (8 * 1)) & 0xFF;
    offset += sizeof(this->motor_right);

    *(outbuffer + offset + 0) = (this->rake_flags >> (8 * 0)) & 0xFF;
    offset += sizeof(this->rake_flags);

    *(outbuffer + offset + 0) = (this->status_motors >> (8 * 0)) & 0xFF;
    offset += sizeof(this->status_motors);

    *(outbuffer + offset + 0) = (this->status_charger >> (8 * 0)) & 0xFF;
    offset += sizeof(this->status_charger);

    *(outbuffer + offset + 0) = (this->status_additions >> (8 * 0)) & 0xFF;
    offset += sizeof(this->status_additions);

    return offset;
  }

  virtual int deserialize(unsigned char *inbuffer)
  {
    int offset = 0;

    this->motor_left =  ((uint16_t) (*(inbuffer + offset)));
    this->motor_left |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
    offset += sizeof(this->motor_left);

    this->motor_right =  ((uint16_t) (*(inbuffer + offset)));
    this->motor_right |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
    offset += sizeof(this->motor_right);

    this->rake_flags =  ((uint8_t) (*(inbuffer + offset)));
    offset += sizeof(this->rake_flags);

    this->status_motors =  ((uint8_t) (*(inbuffer + offset)));
    offset += sizeof(this->status_motors);

    this->status_charger =  ((uint8_t) (*(inbuffer + offset)));
    offset += sizeof(this->status_charger);

    this->status_additions =  ((uint8_t) (*(inbuffer + offset)));
    offset += sizeof(this->status_additions);

    return offset;
  }

  const char * getType(){ return "bb_ioboard/IOToBoard"; };
  const char * getMD5(){ return "4d68e282984326acbe6a6d35fb1c8b97"; };

};
#endif
