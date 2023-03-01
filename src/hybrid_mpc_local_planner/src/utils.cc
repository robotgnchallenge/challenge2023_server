#include <hybrid_mpc_local_planner/utils.hpp>

float AngleLimitPI(float angle)
{
    while (angle > M_PI)
    {
        angle -= 2 * M_PI;
    }
    while (angle <= -M_PI)
    {
        angle += 2 * M_PI;
    }
    return angle;
}

/**
 * @brief 两角差值，限制在[0,pi]
 *
 */
float AngleLimitDiff(float a, float b)
{
    float out = a - b;
    return AngleLimitPI(out);
}