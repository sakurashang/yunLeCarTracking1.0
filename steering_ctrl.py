import math

#Kp=1.0
def PControl(target, current, Kp):
    """
        传入的两个角范围均是0~360，Y轴顺时针方向
    :param target:目标航向角
    :param current:当前航向角
    :param Kp:p参数
    :return:返回相对于target左正右负的角度，左范围0~180°，右范围0~-180°.浮点数
    """
    # 当前航向位于目标航向左边,产生左正右负的误差
    error0 = target - current
    if current<target:
        if error0<180:
            # 在左边
            error=error0
        else:
            # 在右边
            error=-(360-error0)
    else:
        if error0<-180:
            # 在左边
            error=360+error0
        else:
            error=error0

    a = Kp * (error)

    return a

#print(PControl(270,260))


def pure_pursuit(target, current, Lf):
    """
        计算真实转角的值
    :param target:
    :param current:
    :param Lf:
    :return:
    """
    L = 0.9 # 轴距0.9m
    # 计算alpha
    # 当前航向位于目标航向左边,产生左正右负的误差
    error0 = target - current
    if current < target:
        if error0 < 180:
            # 当前航向位于目标航左边，车轮向右转，alpha的值为正
            alpha = error0
        else:
            # 在右边，向左转，alpha的值为负
            alpha = -(360 - error0)
    else:
        # 这里我sb了，想了很就想差了，小于-180度，那就是-190那样的，而不是-10度，简单的一个负数的大小比较，sb了。
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
    global v
    kv = 2
    if kv < 1:
        kv = 1
    if kv > 3.0555:  # 11km/h
        kv = 3.0555
    # https://blog.csdn.net/baoli8425/article/details/116808139 下面公式的原理
    delta = math.atan((2*L*math.sin(alpha/180*math.pi))/kv)  # delta是弧度
    print("delta=", delta)
    # 这里是不是限制了转弯最大度数是27度
    limit = 27/180*math.pi
    if delta > limit:
        delta = limit
    if delta < -limit:
        delta = -limit
    # 然后这里应该是因为转弯指令数值的范围在正负120，所以又乘120
    cmd_steering = delta/(27/180*math.pi)*120
    # 映射成为转向命令
    return cmd_steering


if __name__ == "__main__":
    # 纯追踪开发
    a = pure_pursuit(-30, 0, 1)
    print(a)
