#ifndef _ROS_SERVICE_GetMiddleRange_h
#define _ROS_SERVICE_GetMiddleRange_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace icreate_lift_navigation
{

static const char GETMIDDLERANGE[] = "icreate_lift_navigation/GetMiddleRange";

  class GetMiddleRangeRequest : public ros::Msg
  {
    public:

    GetMiddleRangeRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return GETMIDDLERANGE; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetMiddleRangeResponse : public ros::Msg
  {
    public:
      typedef float _mid_range_type;
      _mid_range_type mid_range;

    GetMiddleRangeResponse():
      mid_range(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_mid_range;
      u_mid_range.real = this->mid_range;
      *(outbuffer + offset + 0) = (u_mid_range.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mid_range.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mid_range.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mid_range.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mid_range);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_mid_range;
      u_mid_range.base = 0;
      u_mid_range.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mid_range.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mid_range.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mid_range.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mid_range = u_mid_range.real;
      offset += sizeof(this->mid_range);
     return offset;
    }

    const char * getType(){ return GETMIDDLERANGE; };
    const char * getMD5(){ return "8e9bccdc3d20b166c919f0fde3aaa3ab"; };

  };

  class GetMiddleRange {
    public:
    typedef GetMiddleRangeRequest Request;
    typedef GetMiddleRangeResponse Response;
  };

}
#endif
