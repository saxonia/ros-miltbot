#ifndef _ROS_SERVICE_RunTransportation_h
#define _ROS_SERVICE_RunTransportation_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace miltbot_transportation
{

static const char RUNTRANSPORTATION[] = "miltbot_transportation/RunTransportation";

  class RunTransportationRequest : public ros::Msg
  {
    public:
      typedef const char* _mode_type;
      _mode_type mode;

    RunTransportationRequest():
      mode("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_mode = strlen(this->mode);
      varToArr(outbuffer + offset, length_mode);
      offset += 4;
      memcpy(outbuffer + offset, this->mode, length_mode);
      offset += length_mode;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_mode;
      arrToVar(length_mode, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_mode; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_mode-1]=0;
      this->mode = (char *)(inbuffer + offset-1);
      offset += length_mode;
     return offset;
    }

    const char * getType(){ return RUNTRANSPORTATION; };
    const char * getMD5(){ return "e84dc3ad5dc323bb64f0aca01c2d1eef"; };

  };

  class RunTransportationResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    RunTransportationResponse():
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    const char * getType(){ return RUNTRANSPORTATION; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class RunTransportation {
    public:
    typedef RunTransportationRequest Request;
    typedef RunTransportationResponse Response;
  };

}
#endif
