#include "math_lib.hpp"


float linear_scale(float value, float in_min, float in_max, float out_min, float out_max) 
{
    // 防止输入范围无效
    if (in_max == in_min) {
        return out_min; // 或者返回错误值，例如 NAN
    }
    // 线性映射公式
    return out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min);
}

float dead_zone(float value,float dead)
{
  if(value<dead&&value>-dead)
  {
    return 0;
  }
  return value;
}

