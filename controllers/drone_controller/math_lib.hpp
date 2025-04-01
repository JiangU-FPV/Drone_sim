#pragma once

#define m_abs(x) 	((x)>0? (x):(-(x)))
#define m_constrain(x, min, max)	((x>max)?max:(x<min?min:x))

float linear_scale(float value, float in_min, float in_max, float out_min, float out_max);
float dead_zone(float value,float dead);
