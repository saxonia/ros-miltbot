#ifndef _ROS_SERVICE_GetWaypointList_h
#define _ROS_SERVICE_GetWaypointList_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "miltbot_common/Waypoint.h"

namespace miltbot_map
{

static const char GETWAYPOINTLIST[] = "miltbot_map/GetWaypointList";

  class GetWaypointListRequest : public ros::Msg
  {
    public:
      typedef const char* _building_type;
      _building_type building;
      typedef const char* _floor_type;
      _floor_type floor;

    GetWaypointListRequest():
      building(""),
      floor("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_building = strlen(this->building);
      varToArr(outbuffer + offset, length_building);
      offset += 4;
      memcpy(outbuffer + offset, this->building, length_building);
      offset += length_building;
      uint32_t length_floor = strlen(this->floor);
      varToArr(outbuffer + offset, length_floor);
      offset += 4;
      memcpy(outbuffer + offset, this->floor, length_floor);
      offset += length_floor;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_building;
      arrToVar(length_building, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_building; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_building-1]=0;
      this->building = (char *)(inbuffer + offset-1);
      offset += length_building;
      uint32_t length_floor;
      arrToVar(length_floor, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_floor; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_floor-1]=0;
      this->floor = (char *)(inbuffer + offset-1);
      offset += length_floor;
     return offset;
    }

    const char * getType(){ return GETWAYPOINTLIST; };
    const char * getMD5(){ return "9b15058aba057dc225e6ed360700b819"; };

  };

  class GetWaypointListResponse : public ros::Msg
  {
    public:
      uint32_t waypoints_length;
      typedef miltbot_common::Waypoint _waypoints_type;
      _waypoints_type st_waypoints;
      _waypoints_type * waypoints;

    GetWaypointListResponse():
      waypoints_length(0), waypoints(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->waypoints_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->waypoints_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->waypoints_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->waypoints_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->waypoints_length);
      for( uint32_t i = 0; i < waypoints_length; i++){
      offset += this->waypoints[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t waypoints_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      waypoints_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      waypoints_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      waypoints_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->waypoints_length);
      if(waypoints_lengthT > waypoints_length)
        this->waypoints = (miltbot_common::Waypoint*)realloc(this->waypoints, waypoints_lengthT * sizeof(miltbot_common::Waypoint));
      waypoints_length = waypoints_lengthT;
      for( uint32_t i = 0; i < waypoints_length; i++){
      offset += this->st_waypoints.deserialize(inbuffer + offset);
        memcpy( &(this->waypoints[i]), &(this->st_waypoints), sizeof(miltbot_common::Waypoint));
      }
     return offset;
    }

    const char * getType(){ return GETWAYPOINTLIST; };
    const char * getMD5(){ return "3c2936debb785d9bd668ee7fc284db7e"; };

  };

  class GetWaypointList {
    public:
    typedef GetWaypointListRequest Request;
    typedef GetWaypointListResponse Response;
  };

}
#endif
