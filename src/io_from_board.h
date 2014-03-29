#ifndef io_from_board_h
#define io_from_board_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

class io_from_board : public ros::Msg
{
  public:
    uint16_t velocity;
    uint8_t status;

  virtual int serialize(unsigned char *outbuffer) const
  {
    int offset = 0;
    *(outbuffer + offset + 0) = (this->velocity >> (8 * 0)) & 0xFF;
    *(outbuffer + offset + 1) = (this->velocity >> (8 * 1)) & 0xFF;
    offset += sizeof(this->velocity);
    *(outbuffer + offset + 0) = (this->status >> (8 * 0)) & 0xFF;
    offset += sizeof(this->status);
    return offset;
  }

  virtual int deserialize(unsigned char *inbuffer)
  {
    int offset = 0;
    this->velocity =  ((uint16_t) (*(inbuffer + offset)));
    this->velocity |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
    offset += sizeof(this->velocity);
    this->status =  ((uint8_t) (*(inbuffer + offset)));
    offset += sizeof(this->status);
    return offset;
  }

  const char * getType(){ return "rosserial_avr_tutorial/io_from_board"; };
  const char * getMD5(){ return "93349f6bedeed5145caf4453585c29da"; };

};

#endif