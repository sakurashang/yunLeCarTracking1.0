#  本模块包含手动自动模式切换的功能，包含数据回传的功能。
#  TODO(wen):清理下不用的代码
import serial
import time
import ll2xy
import classOfIMU
import classOfYunLe
import math
import steering_ctrl
import classOfDataRecord
from socket import *
import struct
import threading

from ctrl_algorithm.pid_method import PID

#  TODO(wen):把这两个合成一个函数模块，功能为返回utm格式路径点
#  读取utm格式路径点
path_x_y = []
with open('pathGenerate/记录下的要跟踪的轨迹点2023-06-04 21:04:31.csv', 'r')as fileopen:
    fileopen.readline()  # 去表头
    content = fileopen.readlines()
    for msg_line in content:
        # 得到经纬度，然后转换成path_x_y
        msg_line_list = msg_line.split(',')
        # ciee_test.csv
        y = float(msg_line_list[7])
        x = float(msg_line_list[6])  # weiDu
        path_x_y.append((x, y))
    fileopen.close()
print('点加载完成')
"""增加读取经纬度数据的功能"""



#  初始化小车控制参数
headingAngle = 0
x, y = 0, 0
v = 0
Kp = 1.0  # PID参数之一



def is_arrive(vehicle_x_y, path_x_y, i):
    """
        纯追踪判断小车是否逻辑到达目标点，方便切换目标点用
    :param vehicle_x_y:
    :param path_x_y:
    :param i:
    :return:
    """
    global v
    # 计算小车与当前目标点的距离
    goal_point = path_x_y[i]
    distance = math.sqrt((vehicle_x_y[0] - goal_point[0]) ** 2 + (vehicle_x_y[1] - goal_point[1]) ** 2)
    print("离路径点" + str(i) + "的距离为：" + str(distance))

    # 计算与下一个跟踪点的距离
    next_goal_point = path_x_y[i + 1]
    distance2net_point = math.sqrt((vehicle_x_y[0] - next_goal_point[0]) ** 2 + (vehicle_x_y[1] - next_goal_point[1]) ** 2)

    if (distance < 1) or (distance2net_point < distance):  # 说明两点的距离要大于0.8m，直线情况下无所谓，但是转弯情况下就有可能跟踪不上路径。可能会出现转圈的情况
        # 此时认为小车到达了
        return True
    else:
        return False



def calcAngOfY_Axis(vector):
    """
        计算向量与Y轴的顺时针夹角，范围0~360°
    :param vector:
    :return: 返回两个向量的夹角
    """
    #endAng=0
    vector1 = (0, 1)  # Y axis
    vector2 = vector
    AB = [0, 0, vector1[0], vector1[1]]
    CD = [0, 0, vector2[0], vector2[1]]

    def angle(v1, v2):
        # 计算v1，v2两角的0~180度角度
        dx1 = v1[2] - v1[0]
        dy1 = v1[3] - v1[1]
        dx2 = v2[2] - v2[0]
        dy2 = v2[3] - v2[1]
        # atan2和atan返回的都是以弧度为单位，结果是在 -pi 和 pi 之间，但是atan2时传入两个参数，而atan是传入（y/x)这一个参数，可能存在x为零的情况
        # y轴转换成角度的值为90度
        # https://blog.csdn.net/weixin_43229030/article/details/108636547 atan2的弧度值范围
        angle1 = math.atan2(dy1, dx1)
        #将弧度转为角度
        angle1 = angle1 * 180 / math.pi
        # print(angle1)
        angle2 = math.atan2(dy2, dx2)
        angle2 = angle2 * 180 / math.pi
        # print(angle2)
        if angle1 * angle2 >= 0:
            included_angle = abs(angle1 - angle2)
        else:
            included_angle = abs(angle1) + abs(angle2)
            if included_angle > 180:
                included_angle = 360 - included_angle
        return included_angle

    ang1 = angle(AB, CD)
    # 浮点数应该可以比较大小，但是比较相等要用精度判断
    # if waypoint_x - x > 0:
    #     # 在右边，0~180
    #     endAng = ang1
    # else:
    #     # 在左边，180~360
    #     endAng = 360 - ang1
    # 这里还要进行一次判断的原因是，我们最后要求的值是从y轴顺时针方西和向量的夹角，范围0-360，那么如果x为负时，那么我们一开始得到的angel1，是逆时针的值，小于180，要变成顺时针获得
    # 的，就要用360再减一次
    if vector2[0] < 0:
        ang1 = 360-ang1
    print("小车指向目标点向量与Y轴夹角=", ang1)
    return ang1


def PControl(target, current):
    """
        P控制器
    :param target:
    :param current:
    :return:
    """
    a = Kp * (target - current * 3.6)

    return a


if __name__ == "__main__":
    # 室外路面建议2公里以上
    target_speed = 1.8  # 单位为km/h

    # 初始化数据上传模块、Imu、小车
    imu = classOfIMU.Imu()
    car = classOfYunLe.Car(1)
    path = r'pathRecord/'
    record = classOfDataRecord.DataRecord(path + "行驶时记录数据" +
                                          time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())) +
                                          ".csv")

    cmd_steering = 0
    cmd_v = 0.5
    Lf = 1.0  # look-ahead distance

    #  初始化第一个跟踪的路径点
    i = 0
    length = len(path_x_y)

    while i <= length-2:  # 当跟踪的最后一个点时候，结束跟踪，小车制动。因为每步只跟踪一个点，所以i的范围可以从0~length-1,设为len-2保险
        """全部处理完再合成指令发送"""
        waypoint = path_x_y[i]

        GPSWeek, GPSTime, imu_lon, imu_lat, imu_altitude,headingAngle, x, y, v, imu_gps_state, imu_satellite_num, imu_warning = imu.stateOfCar()  # 执行一次采样一次。这里用imu.stateOfCar实现
        print(headingAngle, x, y, v)
        car.receMsg()
        vehicle_x_y = (x, y)
        # 计算小车是否到达跟踪点
        if is_arrive(vehicle_x_y, path_x_y, i) == False:
            # TODO 进行横向跟踪控制，将控制指令的产生用函数来实现
            # 判断小车航向角是否满足要求:
            # bangbang:小车位置与跟踪点构成的向量与正北顺时针夹角-小车航向角，若>0且差值大于精度，则小车右转;反之，小车左转
            # 用pid实现

            # 先判断向量的象限，再计算目标夹角

            # 计算小车指向目标点的向量
            car2goalPoint = (waypoint[0]-x, waypoint[1]-y)
            # 获得小车位置与跟踪点构成的向量与正北顺时针夹角
            goalHeadingAng = calcAngOfY_Axis(car2goalPoint)
            #etc = goalAng - headingAngle


            # 转向控制pid实现，需要调参
            #target_headingAngle=0 #小车指向目标点航向角
            #cmd_steering=PControl(goalHeadingAng,headingAngle)   #这里应该是需要转换角度到相同的起始部位。heading是正北顺时针，goalAng方向是目标方向，是小车指向目标点的方向。
            cmd_steering = steering_ctrl.pure_pursuit(goalHeadingAng, headingAngle, Lf)
            print('goalHeadingAng-headingAngle=', goalHeadingAng-headingAngle)
            #cmd_steering = headingAngle+cmd_steering
            #cmd_steering = cmd_steering

            # 限定在小车接收范围内，+—120°以内，精度0.1。并且误差在60°以上时就方向盘打满.在60°以下进行缩放
            if cmd_steering > 60:
                cmd_steering = 120.0
            elif cmd_steering < -60:
                cmd_steering = -120.0
            else:
                cmd_steering = cmd_steering*2.0
            # 以上，符合条件的转角命令生成完毕

            # 速度控制pid实现
            P = 0.35
            I = 0.6
            D = 0.05
            pid = PID(P, I, D)
            currentSpeed = v * 3.6
            pid.setValue = 1.8  # 设定目标值
            cmd_v = pid.pidCalculate(currentSpeed)
            # 直接给定速度
            # cmd_v = 4/3.6

        else:
            # 更新跟踪点为下一个点
            i = i + 1

        # 设定一个速度阈值，保障安全
        if cmd_v > 3.6:  # 3.6km/h限定
            cmd_v = 3.6
        if cmd_v <= 0.0:
            cmd_v = 0.0
        print('cmd_steering=', cmd_steering, 'cmd_v', cmd_v)
        car.sendMsg(1, 1, cmd_steering, cmd_v, 0)
        time.sleep(1)

        # 记录状态。需要记录的数据字段：GPSweek，GPStime，经度，纬度，航向角，x,y,小车发送的指令（转角，速度）.一共九个。
        # 需求：再增加一个字段，存跟踪的i
        # GPSWeek, GPSTime, imu_lon, imu_lat, headingAngle, x, y, v, cmd_steering, cmd_v
        msg = [GPSWeek, GPSTime, imu_lon, imu_lat, imu_altitude,headingAngle, x, y, v, cmd_steering, cmd_v,imu_gps_state,imu_satellite_num,imu_warning]
        record.data_record(msg)

    # 最后需要停下来，制动
    car.sendMsg(1, 2, 0.0, 0, 1)
    del record

    while 1:
        print('死循环等待')
        time.sleep(1)

