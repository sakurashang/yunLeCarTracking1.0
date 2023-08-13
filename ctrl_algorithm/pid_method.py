# @File         : pid_method.py
# @Project      : yunLeCarTrackingFunc_30
# @Time         : 2022/1/8 下午9:29
# @Author       : Kevin
# @Style        : Google Open Source Project Style
# @Description  : 位置式PID算法，有参考价值，来源：https://www.cnblogs.com/wangmantou/p/12787288.html

# TODO:对该代码进行解析，移植到车辆控制代码中
import time
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import make_interp_spline as spline


class PID:
    def __init__(self, P=0.2, I=0.0, D=0.0):
        self.kp = P
        self.ki = I
        self.kd = D

        self.setValue = 0

        self.lastErr = 0
        self.errSum = 0
        self.errSumLimit = 10

    def pidCalculate(self, curValue):
        curValue = curValue
        err = self.setValue - curValue
        dErr = err - self.lastErr
        self.lastErr = err

        self.errSum += err
        if (self.errSum < -self.errSumLimit):
            self.errSum = -self.errSumLimit
        elif (self.errSum > self.errSumLimit):
            self.errSum = self.errSumLimit

        self.output = self.kp * err + (self.ki * self.errSum) + (self.kd * dErr)
        return self.output


def learn_pid(P=0.2, I=0.0, D=0.0, Len=100):
    # 初始化PID对象
    pid = PID(P, I, D)
    pid.setValue = 0    # 设定目标值
    curValue = 0

    curValueList = []
    timeList = []
    setValueList = []

    for i in range(1, Len):
        out = pid.pidCalculate(curValue)
        curValue = out
        if i > 9:
            pid.setValue = 1.8  # 设定阶跃信号
        time.sleep(0.01)

        curValueList.append(curValue)
        setValueList.append(pid.setValue)   # 激励信号存储
        timeList.append(i)
    print(curValueList)
    timeSm = np.array(timeList)
    timeSmooth = np.linspace(timeSm.min(), timeSm.max(), 300)  # 将x轴300等分
    curValueSmooth = spline(timeList, curValueList)(timeSmooth)  # 插值.使原y轴数据平滑
    plt.figure(0)
    plt.plot(timeSmooth, curValueSmooth)
    plt.plot(timeList, setValueList)
    plt.xlim((0, Len))
    plt.ylim((min(curValueList) - 0.5, max(curValueList) + 0.5))
    plt.xlabel('time (s)')
    plt.ylabel('PID (PV)')
    plt.title('TEST PID')

   # plt.ylim((1 - 0.5, 1 + 0.5))

    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    learn_pid(0.35, 0.6, 0.05, Len=80)
    #learn_pid(0.7, 0.2, 0.1, Len=80)