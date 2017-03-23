#ifndef _ROS_SERVICE_ViewTargetQueue_h
#define _ROS_SERVICE_ViewTargetQueue_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "miltbot_common/Waypoint.h"

namespace miltbot_system
{

static const char VIEWTARGETQUEUE[] = "miltbot_system/ViewTargetQueue";

  class ViewTargetQueueRequest : public ros::Msg
  {
    public:

    ViewTargetQueueRequest()
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

    const char * getType(){ return VIEWTARGETQUEUE; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class ViewTargetQueueResponse : public ros::Msg
  {
    public:
      uint32_t target_queue_length;
      typedef miltbot_common::Waypoint _target_queue_type;
      _target_queue_type st_target_queue;
      _target_queue_type * target_queue;

    ViewTargetQueueResponse():
      target_queue_length(0), target_queue(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->target_queue_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->target_queue_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->target_queue_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->target_queue_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->target_queue_length);
      for( uint32_t i = 0; i < target_queue_length; i++){
      offset += this->target_queue[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t target_queue_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      target_queue_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      target_queue_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      target_queue_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->target_queue_length);
      if(target_queue_lengthT > target_queue_length)
        this->target_queue = (miltbot_common::Waypoint*)realloc(this->target_queue, target_queue_lengthT * sizeof(miltbot_common::Waypoint));
      target_queue_length = target_queue_lengthT;
      for( uint32_t i = 0; i < target_queue_length; i++){
      offset += this->st_target_queue.deserialize(inbuffer + offset);
        memcpy( &(this->target_queue[i]), &(this->st_target_queue), sizeof(miltbot_common::Waypoint));
      }
     return offset;
    }

    const char * getType(){ return VIEWTARGETQUEUE; };
    const char * getMD5(){ return "74943cf732ebeb4f092b5bfe45b72371"; };

  };

  class ViewTargetQueue {
    public:
    typedef ViewTargetQueueRequest Request;
    typedef ViewTargetQueueResponse Response;
  };

}
#endif
