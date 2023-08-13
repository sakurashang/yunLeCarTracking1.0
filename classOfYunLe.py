"""进行云乐小车的类型声明"""
# 小车要控制它，需要的信息如下：小车的属性和对它的操作，属性分别是五个，操作分别是发送和接收信息,发送和接收信息应该是接口
#   SCU_Drive_Mode_Req,SCU_ShiftLevel_Req,SCU_Steering_Wheel_Angle,SCU_Target_Speed,SCU_Brk_En五个参数在发送的时候需要
import datetime
import json
import cantools
import os
import time
import can
import math
import classOfIMU
from ctrl_algorithm.pid_method import PID

# 全局变量，dbc文件解析和can0通信初始化
dbcPath = os.path.join(os.getcwd(), "YunleCANAfix.dbc")
db = cantools.db.load_file(dbcPath)

can_bus = can.interface.Bus('can0', bustype='socketcan', bitrate=500000)


class Car:
    # 实例化节点的时候默认开启
    def __init__(self, Car_Switch):
        """
        :param Car_Switch: Car_Switch=1 #默认为1，开启小车控制功能
        """
        self.Car_Switch = Car_Switch
        #sendMsg用的task
        self.task = self.__init_sendMsg(1, 2, 0, 0, 0)

        # 2021/01/21 新增误差修正功能，小车开启对象并且转角不变化后，记录误差项并提供给后续命令的发送。这里默认小车前轮转角在上电后2秒内达到稳态
        time.sleep(2)
        vehicle_sts = self.receMsg()
        if vehicle_sts['Steering_Wheel_Direction'] == 0:
            # 左
            vehicle_sts['CCU_Steering_Wheel_Angle'] = -vehicle_sts['CCU_Steering_Wheel_Angle']

        # 误差
        self.error = vehicle_sts['CCU_Steering_Wheel_Angle']
        print('初始化误差是', str(self.error))



    def __msgInit(self,SCU_Drive_Mode_Req,SCU_ShiftLevel_Req,SCU_Steering_Wheel_Angle,SCU_Target_Speed,SCU_Brk_En):
        """
            初始化消息处理：处理dbc文件
        :return:返回message，给send_period、modify作为其参数
        """
        scu_message = db.get_message_by_name('SCU')

        scuData = {}
        for i in scu_message.signal_tree:
            scuData[i] = 0
        scuData["SCU_Drive_Mode_Req"] = SCU_Drive_Mode_Req  # 模式标志位, 0:-不影响; 1:-自动驾驶模式请求（0x120）; 2:-驾驶员-PAD模式请求（0x100）;3：-驾驶员-方向盘模式请求（默认）
        scuData["SCU_ShiftLevel_Req"] = SCU_ShiftLevel_Req  ##挡位，D前：1; N空：2; R倒：3
        scuData["SCU_Steering_Wheel_Angle"] = SCU_Steering_Wheel_Angle  # 转向角度，±120.0（0.1）实际上发送的命令可以超过这个范围
        scuData["SCU_Target_Speed"] = SCU_Target_Speed  # 目标速度, 0.0-20.0km/h（0.1）
        scuData["SCU_Brk_En"] = SCU_Brk_En  # 是否刹车，否：0；是：1

        data = scu_message.encode(scuData)
        # 得到发送的帧id
        print("=================================")
        print(scu_message.frame_id)
        print(scu_message.senders)
        # scu_message.frame_id=0x51


        message = can.Message(arbitration_id=scu_message.frame_id, data=data, is_extended_id=False)

        return message


    def __init_sendMsg(self,SCU_Drive_Mode_Req,SCU_ShiftLevel_Req,SCU_Steering_Wheel_Angle,SCU_Target_Speed,SCU_Brk_En):
        """
            (调用modify函数实现)小车会以50ms周期一直发送该指令，直到发送下一指令或函数Stop_Flag=0
        :param SCU_Drive_Mode_Req:
        :param SCU_ShiftLevel_Req:
        :param SCU_Steering_Wheel_Angle:
        :param SCU_Target_Speed:
        :param SCU_Brk_En:
        :return:返回task，方便调用后续调用modify
        """
        if self.Car_Switch == 0:
            print('当前节点为信息读取节点')
        else:
            # 2021.4.5 速度安全限制
            if SCU_Target_Speed >=10.0:
                SCU_Target_Speed = 10.0
                print("warning：触发速度安全限制10km/h,from classOfYunLe.py")

            # 转换成CAN帧，发送
            message = self.__msgInit(SCU_Drive_Mode_Req, SCU_ShiftLevel_Req, SCU_Steering_Wheel_Angle, SCU_Target_Speed, SCU_Brk_En)

            print('发送的信息：', message)
            task = can_bus.send_periodic(message, period=0.05, duration=None, store_task=True)
            print('已开启sendperiod线程')
            # 假设task线程在本函数结束以后不停止
            return task

    def sendMsg(self, SCU_Drive_Mode_Req, SCU_ShiftLevel_Req, SCU_Steering_Wheel_Angle, SCU_Target_Speed, SCU_Brk_En):
        """
            发送车辆控制指令
        :param SCU_Drive_Mode_Req:
        :param SCU_ShiftLevel_Req:
        :param SCU_Steering_Wheel_Angle:
        :param SCU_Target_Speed:
        :param SCU_Brk_En:
        :return:
        """
        SCU_Steering_Wheel_Angle = SCU_Steering_Wheel_Angle - self.error
        message = self.__msgInit(SCU_Drive_Mode_Req, SCU_ShiftLevel_Req, SCU_Steering_Wheel_Angle,
                               SCU_Target_Speed, SCU_Brk_En)
        self.task.modify_data(message)
        pass

    def receMsg(self):
        # 回调函数
        get_data = can_bus.recv()   # recv()函数只管接收信息，会把CAN总线中所有的信息读取进来

        # 设计成当读到0x51的时候返回，否则继续读   /0x51帧是车辆的信息状态
        while 1:
            if get_data.arbitration_id == 0x51:  # 在这里进行信息的分拣
                #print("接收到了车辆状态信息")
                vehicle_sts = db.decode_message(get_data.arbitration_id, get_data.data)
                print(vehicle_sts)
                print("=================",vehicle_sts["CCU_Vehicle_Speed"])
                return vehicle_sts
            else:
                get_data = can_bus.recv()
                # 当获取的状态改变时再更新信息
            pass

    @staticmethod
    def recv_msg_0x2AA():
        get_data = can_bus.recv()
        while 1:
            if get_data.arbitration_id == 0x2AA:
                BMU_System_Info = db.decode_message(get_data.arbitration_id, get_data.data)
                print(BMU_System_Info)
                #time.sleep(0.1)
                return BMU_System_Info
            else:
                get_data = can_bus.recv()


if __name__ == "__main__":
    
    car = Car(1)
    car_rec = Car(0)
    imu = classOfIMU.Imu()
    v = 0.1

    #  读取utm格式路径点
    path_x_y = []
    path_alt = []
    path_v = []
    path_cmd_v = []
    with open('pathRecord/', 'r') as fileopen:
        fileopen.readline()  # 去表头
        content = fileopen.readlines()
        for msg_line in content:
            # 得到经纬度，然后转换成path_x_y
            msg_line_list = msg_line.split(',')
            # ciee_test.csv
            alt = float(msg_line_list[4])
            y = float(msg_line_list[7])
            x = float(msg_line_list[6])  # weiDu
            v = float(msg_line_list[8])
            cmd_v = float(msg_line_list[10])
            path_x_y.append((x, y))
            path_alt.append(alt)
            path_v.append(v)
            path_cmd_v.append(cmd_v)
        fileopen.close()
    print('点加载完成')
    i = 0

    # length = len(path_x_y)
    # while i <= length-2:
    #   #  car_rec.receMsg()
    #     # if v > 3.0:
    #     #     v = v - 0.3
    #     # if v < 0.4:
    #     # # v = v + 0.1
    #     # car.sendMsg(1, 1, 0.0, 0.5, 0)
    #     # time.sleep(5)
    #     # car.sendMsg(1, 1, 0.0, 1.5, 0)
    #     # time.sleep(5)
    #     # car.sendMsg(1, 1, 0.0, 3, 0)
    #     # time.sleep(5)
    #     car.sendMsg(1, 1, 0.0, path_v[i]*1.5, 0)
    #     print("cmd_v = ",path_v[i])
    #     time.sleep(1)
    #     i = i + 1
    #     if i==960:
    #         i=960
    #  #   time.sleep(1)
    # #time.sleep(3)
    while 1:
        GPSWeek, GPSTime, imu_lon, imu_lat, imu_altitude, headingAngle, x, y, v, imu_gps_state, imu_satellite_num, imu_warning = imu.stateOfCar()  # 执行一次采样一次。这里用imu.stateOfCar实现\
        #imu读取现在的高度，从文件中读取后续第四个点的高度，求取两点之间的坡度 imu采样周期 0.15s
        j = i + 4
        alt_future = path_alt[j]
        #横向位移 使用速度乘时间得出
        lateral_displacement = v * 0.6
        #纵向位移
        longitudinal_displacement = abs(imu_altitude - alt_future)
        #求坡度
        a = math.atan2(longitudinal_displacement,lateral_displacement)
        slope = a/math.pi*180
        #用Pid,获得期望加速度
        P = 0.2
        I = 0
        D = 0
        pid = PID(P,I,D)
        pid.setValue(2)
        out = pid.pidCalculate(v)
        #限制pid输出范围
        out = max(min(out,8),-8)
        #对标定表进行查找 输入v,out,slope ,求出扭矩大小

        #发送控制命令
        car.sendMsg(1, 1, 0.0, path_v[i] * 1.5, 0)

        i = i + 1
        if j > len(path_alt):
            break
