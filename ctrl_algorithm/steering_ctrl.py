import math
import time

import ctrl_algorithm.classOfPID as classOfPID


def new_pcontrol(target, current, Kp, Ki, Kd):
    # 引入微分环节
    pid = classOfPID.PID(P=Kp, I=Ki, D=Kd)
    pid.setValue = target  # set end  zz设定值
    outPID = pid.pidPosition(current)
    return outPID


# Kp=1.0
def PControl(target, current, Kp, front_error, Kd,Ki,sum_error):
    """
        传入的两个角范围均是0~360，Y轴顺时针方向
    :param target:目标航向角
    :param current:当前航向角
    :param Kp:p参数
    :return:返回相对于target左正右负的角度，左范围0~180°，右范围0~-180°.浮点数
    """
    # 当前航向位于目标航向左边,产生左正右负的误差
    error0 = target - current
    if current < target:
        if error0 < 180:
            # 在左边
            error = error0
        else:
            # 在右边
            error = -(360 - error0)
    else:
        if error0 < -180:
            # 在左边
            error = 360 + error0
        else:
            error = error0

    steering_angle = Kp * error+Kd*(error-front_error)+Ki*sum_error
    sum_error += error
    return steering_angle, error, sum_error


# print(PControl(270,260))


def pure_pursuit(target, current, Lf):
    """
        计算真实转角的值
    :param target:
    :param current:
    :param Lf:
    :return:
    """
    y_wheelbase = 3.1  # 轴距(y轴轮距)3.1m
    # 计算alpha
    error0 = target - current
    if current < target:
        if error0 < 180:
            # 在左边
            alpha = error0
        else:
            # 在右边
            alpha = -(360 - error0)
    else:
        if error0 < -180:
            # 在左边
            alpha = 360 + error0
        else:
            alpha = error0

    # 限定alpha大小
    if alpha > 90:
        alpha = 90
    if alpha < -90:
        alpha = -90
    # 计算左负右正的delta
    global v  # v是
    kv = 2
    if kv < 1:
        kv = 1
    if kv > 3.0555:  # 11km/h
        kv = 3.0555
    delta = math.atan((2 * y_wheelbase * math.sin(alpha / 180 * math.pi)) / kv)  # delta是弧度
    print("delta=", delta)
    limit = 27 / 180 * math.pi
    if delta > limit:
        delta = limit
    if delta < -limit:
        delta = -limit
    cmd_steering = delta / (27 / 180 * math.pi) * 120
    # 映射成为转向命令
    return cmd_steering


if __name__ == "__main__":
    # 纯追踪开发
    # a = pure_pursuit(-30, 0, 1)
    a = PControl(-10, 0, 1.0)
    print(a)

    a_list = []
    a = 0
    start = time.time()
    while True:
        time.sleep(0.1)
        a = new_pcontrol(target=1, current=a, Kp=10, Ki=0, Kd=0)
        a_list.append(a)
        end = time.time()
        if end - start > 10:
            break

    print(a_list)

    # a = new_pcontrol(-0.5, 0, 1, 0, 1)
    # print(a)
    #
    # a = new_pcontrol(a, 0, 1, 0, 1)
    # print(a)
