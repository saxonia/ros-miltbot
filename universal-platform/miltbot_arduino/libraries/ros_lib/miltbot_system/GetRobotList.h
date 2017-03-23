#ifndef _ROS_SERVICE_GetRobotList_h
#define _ROS_SERVICE_GetRobotList_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace miltbot_system
{

static const char GETROBOTLIST[] = "miltbot_system/GetRobotList";

  class GetRobotListRequest : public ros::Msg
  {
    public:

    GetRobotListRequest()
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

    const char * getType(){ return GETROBOTLIST; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetRobotListResponse : public ros::Msg
  {
    public:
      uint32_t robot_list_length;
      typedef char* _robot_list_type;
      _robot_list_type st_robot_list;
      _robot_list_type * robot_list;

    GetRobotListResponse():
      robot_list_length(0), robot_list(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->robot_list_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->robot_list_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->robot_list_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->robot_list_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->robot_list_length);
      for( uint32_t i = 0; i < robot_list_length; i++){
      uint32_t length_robot_listi = strlen(this->robot_list[i]);
      varToArr(outbuffer + offset, length_robot_listi);
      offset += 4;
      memcpy(outbuffer + offset, this->robot_list[i], length_robot_listi);
      offset += length_robot_listi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t robot_list_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      robot_list_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      robot_list_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      robot_list_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->robot_list_length);
      if(robot_list_lengthT > robot_list_length)
        this->robot_list = (char**)realloc(this->robot_list, robot_list_lengthT * sizeof(char*));
      robot_list_length = robot_list_lengthT;
      for( uint32_t i = 0; i < robot_list_length; i++){
      uint32_t length_st_robot_list;
      arrToVar(length_st_robot_list, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_robot_list; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_robot_list-1]=0;
      this->st_robot_list = (char *)(inbuffer + offset-1);
      offset += length_st_robot_list;
        memcpy( &(this->robot_list[i]), &(this->st_robot_list), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return GETROBOTLIST; };
    const char * getMD5(){ return "af05f46b3c5dc43342331c0cab33e567"; };

  };

  class GetRobotList {
    public:
    typedef GetRobotListRequest Request;
    typedef GetRobotListResponse Response;
  };

}
#endif
