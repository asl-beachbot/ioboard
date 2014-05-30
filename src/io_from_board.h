#ifndef io_from_board_h
#define io_from_board_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

class io_from_board : public ros::Msg
{
  public:
  
  int32_t deltaUmLeft;
  int32_t deltaUmRight;
  uint16_t mVBattery;
  uint32_t timestamp;

  virtual int serialize(unsigned char *outbuffer) const
  {
    int offset = 0;

    union {
      int32_t real;
      uint32_t base;
    } u_deltaUmLeft;
    u_deltaUmLeft.real = this->deltaUmLeft;
    *(outbuffer + offset + 0) = (u_deltaUmLeft.base >> (8 * 0)) & 0xFF;
    *(outbuffer + offset + 1) = (u_deltaUmLeft.base >> (8 * 1)) & 0xFF;
    *(outbuffer + offset + 2) = (u_deltaUmLeft.base >> (8 * 2)) & 0xFF;
    *(outbuffer + offset + 3) = (u_deltaUmLeft.base >> (8 * 3)) & 0xFF;
    offset += sizeof(this->deltaUmLeft);

    union {
      int32_t real;
      uint32_t base;
    } u_deltaUmRight;
    u_deltaUmRight.real = this->deltaUmRight;
    *(outbuffer + offset + 0) = (u_deltaUmRight.base >> (8 * 0)) & 0xFF;
    *(outbuffer + offset + 1) = (u_deltaUmRight.base >> (8 * 1)) & 0xFF;
    *(outbuffer + offset + 2) = (u_deltaUmRight.base >> (8 * 2)) & 0xFF;
    *(outbuffer + offset + 3) = (u_deltaUmRight.base >> (8 * 3)) & 0xFF;
    offset += sizeof(this->deltaUmRight);

    *(outbuffer + offset + 0) = (this->mVBattery >> (8 * 0)) & 0xFF;
    *(outbuffer + offset + 1) = (this->mVBattery >> (8 * 1)) & 0xFF;
    offset += sizeof(this->mVBattery);

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

    union {
      int32_t real;
      uint32_t base;
    } u_deltaUmLeft;
    u_deltaUmLeft.base = 0;
    u_deltaUmLeft.base =  ((uint32_t) (*(inbuffer + offset)));
    u_deltaUmLeft.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
    u_deltaUmLeft.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
    u_deltaUmLeft.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
    this->deltaUmLeft = u_deltaUmLeft.real;    
    offset += sizeof(this->deltaUmLeft);

    union {
      int32_t real;
      uint32_t base;
    } u_deltaUmRight;
    u_deltaUmRight.base = 0;
    u_deltaUmRight.base =  ((uint32_t) (*(inbuffer + offset)));
    u_deltaUmRight.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
    u_deltaUmRight.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
    u_deltaUmRight.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
    this->deltaUmRight = u_deltaUmRight.real;
    offset += sizeof(this->deltaUmRight);

    this->mVBattery =  ((uint16_t) (*(inbuffer + offset)));
    this->mVBattery |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
    offset += sizeof(this->mVBattery);

    this->timestamp =  ((uint32_t) (*(inbuffer + offset)));
    this->timestamp |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
    this->timestamp |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
    this->timestamp |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
    offset += sizeof(this->timestamp);

    return offset;
  }

  const char * getType(){ return "ioboard/IOFromBoard"; };
  const char * getMD5(){ return "45bbdad0f6e3dfc7d7db5c507a72028d"; };

};

#endif
