#include <cmath>
#include <hybrid_mpc_local_planner/pid.hpp>
#include <hybrid_mpc_local_planner/utils.hpp>

PID::PID()
{
    kp = 1;
    ki = 0;
    kd = 0;
    int_item = 0, int_duty = 1;
    now_err = 0, last_err = 0, last_last_err = 0;
    output = 0, delta_output = 0;
    OUTPUT_MAX = 999;
    INT_ITEM_MAX = 1;
    CTRL_DEAD_TH = 0;
}

PID::PID(float _kp, float _ki, float _kd, float _int_duty,
         float _OUTPUT_MAX, float _INT_ITEM_MAX, float _CTRL_DEAD_TH)
{
    kp = _kp;
    ki = _ki;
    kd = _kd;
    int_item = 0, int_duty = _int_duty;
    now_err = 0, last_err = 0, last_last_err = 0;
    output = 0, delta_output = 0;
    OUTPUT_MAX = _OUTPUT_MAX;
    INT_ITEM_MAX = _INT_ITEM_MAX;
    CTRL_DEAD_TH = _CTRL_DEAD_TH;
}

float PID::calcuOutput(float now_value, float desired_value)
{
    now_err = desired_value - now_value;
    int_item += now_err * int_duty;
    __LIMIT(int_item, INT_ITEM_MAX);
    output = kp * now_err + ki * int_item + kd * last_err;
    if (fabs(now_err) <= CTRL_DEAD_TH)
    {
        output = 0;
    }
    last_err = now_err;
    __LIMIT(output, OUTPUT_MAX);
    return output;
}

float PID::calcuDeltaOutput(float now_value, float desired_value)
{
    last_last_err = last_err;
    last_err = now_err;
    now_err = desired_value - now_value;
    delta_output = kp * (now_err - last_err) + ki * now_err + kd * (now_err - 2 * last_err + last_last_err);
    return now_value + delta_output;
}

void PID::setParam(float _kp, float _ki, float _kd)
{
    kp = _kp;
    ki = _ki;
    kd = _kd;
}
