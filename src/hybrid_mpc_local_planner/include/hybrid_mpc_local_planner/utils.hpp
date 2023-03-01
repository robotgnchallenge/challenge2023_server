#include <cmath>


#define __LIMIT(value, max) \
    if (value > max)        \
        value = max;        \
    else if (value < -max)  \
    value = -max

#define __LIMIT_FROM_TO(value, min, max) \
    {                                    \
        if ((value) > (max))             \
            value = max;                 \
        if ((value) < (min))             \
            value = min;                 \
    }

//角度制转化为弧度制
#define __ANGLE2RAD(x) (((x)*1.0) / 180.0f * PI)
//弧度制转换为角度制
#define __RAD2ANGLE(x) (((x)*1.0) / PI * 180.0f)


float AngleLimitPI(float angle);
float AngleLimitDiff(float a, float b);