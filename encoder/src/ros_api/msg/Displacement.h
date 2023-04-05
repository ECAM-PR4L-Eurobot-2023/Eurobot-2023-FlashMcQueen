#ifndef _ROS_eurobot2023_Displacement_h
#define _ROS_eurobot2023_Displacement_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace eurobot2023
{

  class Displacement : public ros::Msg
  {
    public:
      typedef float _angle_start_type;
      _angle_start_type angle_start;
      typedef float _x_type;
      _x_type x;
      typedef float _y_type;
      _y_type y;
      typedef float _angle_end_type;
      _angle_end_type angle_end;
      typedef bool _backward_type;
      _backward_type backward;

    Displacement():
      angle_start(0),
      x(0),
      y(0),
      angle_end(0),
      backward(0)
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
      } u_x;
      u_x.real = this->x;
      *(outbuffer + offset + 0) = (u_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x);
      union {
        float real;
        uint32_t base;
      } u_y;
      u_y.real = this->y;
      *(outbuffer + offset + 0) = (u_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y);
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
        bool real;
        uint8_t base;
      } u_backward;
      u_backward.real = this->backward;
      *(outbuffer + offset + 0) = (u_backward.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->backward);
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
      } u_x;
      u_x.base = 0;
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x = u_x.real;
      offset += sizeof(this->x);
      union {
        float real;
        uint32_t base;
      } u_y;
      u_y.base = 0;
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->y = u_y.real;
      offset += sizeof(this->y);
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
        bool real;
        uint8_t base;
      } u_backward;
      u_backward.base = 0;
      u_backward.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->backward = u_backward.real;
      offset += sizeof(this->backward);
     return offset;
    }

    const char * getType(){ return "eurobot2023/Displacement"; };
    const char * getMD5(){ return "9c3e0cb7af3152d38417f4ac3631ef91"; };

  };

}
#endif