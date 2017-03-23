#ifndef _ROS_SERVICE_SetMap_h
#define _ROS_SERVICE_SetMap_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace miltbot_map
{

static const char SETMAP[] = "miltbot_map/SetMap";

  class SetMapRequest : public ros::Msg
  {
    public:
      typedef const char* _floor_type;
      _floor_type floor;
      typedef int32_t _target_number_type;
      _target_number_type target_number;

    SetMapRequest():
      floor(""),
      target_number(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_floor = strlen(this->floor);
      varToArr(outbuffer + offset, length_floor);
      offset += 4;
      memcpy(outbuffer + offset, this->floor, length_floor);
      offset += length_floor;
      union {
        int32_t real;
        uint32_t base;
      } u_target_number;
      u_target_number.real = this->target_number;
      *(outbuffer + offset + 0) = (u_target_number.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_target_number.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_target_number.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_target_number.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->target_number);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_floor;
      arrToVar(length_floor, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_floor; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_floor-1]=0;
      this->floor = (char *)(inbuffer + offset-1);
      offset += length_floor;
      union {
        int32_t real;
        uint32_t base;
      } u_target_number;
      u_target_number.base = 0;
      u_target_number.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_target_number.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_target_number.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_target_number.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->target_number = u_target_number.real;
      offset += sizeof(this->target_number);
     return offset;
    }

    const char * getType(){ return SETMAP; };
    const char * getMD5(){ return "94583ec6ebf756fc69a275db31ce6eb7"; };

  };

  class SetMapResponse : public ros::Msg
  {
    public:
      typedef bool _flag_type;
      _flag_type flag;

    SetMapResponse():
      flag(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_flag;
      u_flag.real = this->flag;
      *(outbuffer + offset + 0) = (u_flag.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->flag);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_flag;
      u_flag.base = 0;
      u_flag.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->flag = u_flag.real;
      offset += sizeof(this->flag);
     return offset;
    }

    const char * getType(){ return SETMAP; };
    const char * getMD5(){ return "24842bc754e0f5cc982338eca1269251"; };

  };

  class SetMap {
    public:
    typedef SetMapRequest Request;
    typedef SetMapResponse Response;
  };

}
#endif
