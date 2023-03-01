import math


def angle_limit_pi(angle):
    """limit an angle to [0,pi]"""
    while (angle > math.pi):
        angle -= 2 * math.pi
    while (angle <= -math.pi):
        angle += 2 * math.pi
    return angle


class PID:
    def __init__(self, kp, ki, kd, int_duty=0.01, int_max=100, output_max=10, sub_ctrl=False):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.now_err = 0  # 当前偏差
        self.last_err = 0  # 上一轮偏差
        self.last_last_err = 0  # 上上轮偏差
        self.delta_ctrl_value = 0  # 增量式PID的控制增量
        self.int_item = 0  # 积分量
        self.int_duty = int_duty  # 积分周期/s
        self.INT_MAX = int_max  # 积分限额
        self.OUTPUT_MAX = output_max  # 控制量限额
        self.sub_ctrl = sub_ctrl  # 是否开启分段pid

    def calcu_delta_output(self, now_value, expect_value):
        """Increment PID"""
        self.last_last_err = self.last_err
        self.last_err = self.now_err
        self.now_value = now_value
        self.now_err = expect_value - now_value
        self.delta_ctrl_value = self.kp * (self.now_err - self.last_err) + self.ki * \
            self.now_err + self.kd * (self.now_err - 2 * self.last_err
                                      + self.last_last_err)
        return self.now_value + self.delta_ctrl_value

    def calcu_output(self, now_value, expect_value):
        """Position PID"""
        self.now_err = expect_value - now_value
        self.int_item += self.now_err
        if self.int_item >= self.INT_MAX:
            self.int_item = self.INT_MAX
        if self.int_item <= - self.INT_MAX:
            self.int_item = -self.INT_MAX

        output = 0
        if self.sub_ctrl == False:
            output = self.kp * self.now_err + self.ki * \
                self.int_item + self.kd * (self.now_err - self.last_err)
        else:
            if abs(self.now_err) > 0.2:
                output = self.kp * self.now_err + self.ki * \
                    self.int_item + self.kd * (self.now_err - self.last_err)
            else:
                output = 0.05 * self.now_err + 0.15 * \
                    self.int_item + 0.5 * (self.now_err - self.last_err)

        if output > self.OUTPUT_MAX:
            output = self.OUTPUT_MAX
        if output < -self.OUTPUT_MAX:
            output = -self.OUTPUT_MAX
        self.last_err = self.now_err
        return output
