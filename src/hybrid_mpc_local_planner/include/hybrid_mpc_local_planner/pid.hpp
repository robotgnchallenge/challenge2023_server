

class PID
{
private:
    float kp, ki, kd;
    float now_err, last_err, last_last_err;
    float int_item, int_duty;
    float output, delta_output;
    float OUTPUT_MAX, INT_ITEM_MAX;
    float CTRL_DEAD_TH; // control dead threshold

public:
    PID();
    PID(float kp_, float ki_, float kd_, float int_duty_,
        float OUTPUT_MAX_, float INT_ITEM_MAX_, float CTRL_DEAD_TH_);
    void setParam(float kp, float ki, float kd);

    /**
     * @brief Position PID
     *
     * @param now_value
     * @param desired_value
     * @return float
     */
    float calcuOutput(float now_value, float desired_value);

    /**
     * @brief Increment PID
     *
     * @param now_value
     * @param desired_value
     * @return float
     */
    float calcuDeltaOutput(float now_value, float desired_value);

    float getCTRL_DEAD_TH(void) { return CTRL_DEAD_TH; }
};