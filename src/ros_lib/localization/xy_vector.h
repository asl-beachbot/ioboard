#ifndef _ROS_localization_xy_vector_h
#define _ROS_localization_xy_vector_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "localization/xy_point.h"

namespace localization
{

  class xy_vector : public ros::Msg
  {
    public:
      uint8_t points_length;
      localization::xy_point st_points;
      localization::xy_point * points;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = points_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < points_length; i++){
      offset += this->points[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t points_lengthT = *(inbuffer + offset++);
      if(points_lengthT > points_length)
        this->points = (localization::xy_point*)realloc(this->points, points_lengthT * sizeof(localization::xy_point));
      offset += 3;
      points_length = points_lengthT;
      for( uint8_t i = 0; i < points_length; i++){
      offset += this->st_points.deserialize(inbuffer + offset);
        memcpy( &(this->points[i]), &(this->st_points), sizeof(localization::xy_point));
      }
     return offset;
    }

    const char * getType(){ return "localization/xy_vector"; };
    const char * getMD5(){ return "8f02263beef99aa03117a577a3eb879d"; };

  };

}
#endif