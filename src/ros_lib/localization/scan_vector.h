#ifndef _ROS_localization_scan_vector_h
#define _ROS_localization_scan_vector_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "localization/scan_point.h"

namespace localization
{

  class scan_vector : public ros::Msg
  {
    public:
      uint8_t scans_length;
      localization::scan_point st_scans;
      localization::scan_point * scans;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = scans_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < scans_length; i++){
      offset += this->scans[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t scans_lengthT = *(inbuffer + offset++);
      if(scans_lengthT > scans_length)
        this->scans = (localization::scan_point*)realloc(this->scans, scans_lengthT * sizeof(localization::scan_point));
      offset += 3;
      scans_length = scans_lengthT;
      for( uint8_t i = 0; i < scans_length; i++){
      offset += this->st_scans.deserialize(inbuffer + offset);
        memcpy( &(this->scans[i]), &(this->st_scans), sizeof(localization::scan_point));
      }
     return offset;
    }

    const char * getType(){ return "localization/scan_vector"; };
    const char * getMD5(){ return "b83a28963ad6e979b41c22678aaba7cb"; };

  };

}
#endif