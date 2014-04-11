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
    uint32_t timestamp;

  virtual int serialize(unsigned char *outbuffer) const
  {
    int offset = 0;
    *(outbuffer + offset + 0) = (this->velocity >> (8 * 0)) & 0xFF;
    *(outbuffer + offset + 1) = (this->velocity >> (8 * 1)) & 0xFF;
    offset += sizeof(this->velocity);
    *(outbuffer + offset + 0) = (this->status >> (8 * 0)) & 0xFF;
    offset += sizeof(this->status);
    *(outbuffer + offset + 0) = (this->timestamp >> (8 * 0)) & 0xFF;
    *(outbuffer + offset + 1) = (this->timestamp >> (8 * 1)) & 0xFF;
    *(outbuffer + offset + 2) = (this->timestamp >> (8 * 2)) & 0xFF;
    *(outbuffer + offset + 3) = (this->timestamp >> (8 * 3)) & 0xFF;
    offset += sizeof(this->timestamp);
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
    this->velocity =  ((uint32_t) (*(inbuffer + offset)));
    this->velocity |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
    this->velocity |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
    this->velocity |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
    offset += sizeof(this->timestamp);
    return offset;
  }

  const char * getType(){ return "rosserial_avr_tutorial/io_from_board"; };
  const char * getMD5(){ return "93349f6bedeed5145caf4453585c29da"; };

};

#endif