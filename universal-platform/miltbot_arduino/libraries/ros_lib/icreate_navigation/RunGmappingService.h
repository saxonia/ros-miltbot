#ifndef _ROS_SERVICE_RunGmappingService_h
#define _ROS_SERVICE_RunGmappingService_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace icreate_navigation
{

static const char RUNGMAPPINGSERVICE[] = "icreate_navigation/RunGmappingService";

  class RunGmappingServiceRequest : public ros::Msg
  {
    public:
      typedef const char* _task_type;
      _task_type task;

    RunGmappingServiceRequest():
      task("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_task = strlen(this->task);
      varToArr(outbuffer + offset, length_task);
      offset += 4;
      memcpy(outbuffer + offset, this->task, length_task);
      offset += length_task;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_task;
      arrToVar(length_task, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_task; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_task-1]=0;
      this->task = (char *)(inbuffer + offset-1);
      offset += length_task;
     return offset;
    }

    const char * getType(){ return RUNGMAPPINGSERVICE; };
    const char * getMD5(){ return "0ece8f504419f7ca4d91b277e47ff617"; };

  };

  class RunGmappingServiceResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    RunGmappingServiceResponse():
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

    const char * getType(){ return RUNGMAPPINGSERVICE; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class RunGmappingService {
    public:
    typedef RunGmappingServiceRequest Request;
    typedef RunGmappingServiceResponse Response;
  };

}
#endif
