#ifndef _ROS_bbcontrol_TwistWithID_h
#define _ROS_bbcontrol_TwistWithID_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/UInt8.h"

namespace bbcontrol
{

  class TwistWithID : public ros::Msg
  {
    public:
      geometry_msgs::Twist twist;
      std_msgs::UInt8 id;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->twist.serialize(outbuffer + offset);
      offset += this->id.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->twist.deserialize(inbuffer + offset);
      offset += this->id.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "bbcontrol/TwistWithID"; };
    const char * getMD5(){ return "69233daf0e4e1a52c355a014de07098c"; };

  };

}
#endif