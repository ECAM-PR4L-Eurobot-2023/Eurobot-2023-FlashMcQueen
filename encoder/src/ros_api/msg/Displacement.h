#ifndef _ROS_eurobot2023_Displacement_h
#define _ROS_eurobot2023_Displacement_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <ros.h>

namespace msgs
{

  class Displacement : public ros::Msg
  {
    public:
      typedef float _angle_start_type;
      _angle_start_type angle_start;
      typedef float _angle_end_type;
      _angle_end_type angle_end;
      typedef float _distance_type;
      _distance_type distance;

    Displacement():
      angle_start(0),
      angle_end(0),
      distance(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_angle_start;
      u_angle_start.real = this->angle_start;
      *(outbuffer + offset + 0) = (u_angle_start.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angle_start.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angle_start.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angle_start.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angle_start);
      union {
        float real;
        uint32_t base;
      } u_angle_end;
      u_angle_end.real = this->angle_end;
      *(outbuffer + offset + 0) = (u_angle_end.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angle_end.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angle_end.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angle_end.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angle_end);
      union {
        float real;
        uint32_t base;
      } u_distance;
      u_distance.real = this->distance;
      *(outbuffer + offset + 0) = (u_distance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_distance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_distance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_distance.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->distance);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_angle_start;
      u_angle_start.base = 0;
      u_angle_start.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angle_start.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angle_start.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angle_start.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angle_start = u_angle_start.real;
      offset += sizeof(this->angle_start);
      union {
        float real;
        uint32_t base;
      } u_angle_end;
      u_angle_end.base = 0;
      u_angle_end.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angle_end.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angle_end.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angle_end.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angle_end = u_angle_end.real;
      offset += sizeof(this->angle_end);
      union {
        float real;
        uint32_t base;
      } u_distance;
      u_distance.base = 0;
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->distance = u_distance.real;
      offset += sizeof(this->distance);
     return offset;
    }

    const char * getType(){ return "eurobot2023/Displacement"; };
    const char * getMD5(){ return "be88142c5a8be8b0a32c2c8c2f059f00"; };

  };

}
#endif