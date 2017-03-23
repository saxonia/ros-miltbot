#ifndef _ROS_SERVICE_SetRobotState_h
#define _ROS_SERVICE_SetRobotState_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace miltbot_state
{

static const char SETROBOTSTATE[] = "miltbot_state/SetRobotState";

  class SetRobotStateRequest : public ros::Msg
  {
    public:
      typedef const char* _req_type;
      _req_type req;

    SetRobotStateRequest():
      req("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_req = strlen(this->req);
      varToArr(outbuffer + offset, length_req);
      offset += 4;
      memcpy(outbuffer + offset, this->req, length_req);
      offset += length_req;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_req;
      arrToVar(length_req, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_req; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_req-1]=0;
      this->req = (char *)(inbuffer + offset-1);
      offset += length_req;
     return offset;
    }

    const char * getType(){ return SETROBOTSTATE; };
    const char * getMD5(){ return "b8dc53fbc3707f169aa5dbf7b19d2567"; };

  };

  class SetRobotStateResponse : public ros::Msg
  {
    public:
      typedef const char* _res_type;
      _res_type res;

    SetRobotStateResponse():
      res("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_res = strlen(this->res);
      varToArr(outbuffer + offset, length_res);
      offset += 4;
      memcpy(outbuffer + offset, this->res, length_res);
      offset += length_res;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_res;
      arrToVar(length_res, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_res; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_res-1]=0;
      this->res = (char *)(inbuffer + offset-1);
      offset += length_res;
     return offset;
    }

    const char * getType(){ return SETROBOTSTATE; };
    const char * getMD5(){ return "53af918a2a4a2a182c184142fff49b0c"; };

  };

  class SetRobotState {
    public:
    typedef SetRobotStateRequest Request;
    typedef SetRobotStateResponse Response;
  };

}
#endif
