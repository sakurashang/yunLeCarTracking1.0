import classOfIMU
import math

#  读取utm格式路径点
path_x_y = []
path_alt = []
path_v = []
path_cmd_v = []
with open('pathRecord/', 'r')as fileopen:
    fileopen.readline()  # 去表头
    content = fileopen.readlines()
    for msg_line in content:
        # 得到经纬度，然后转换成path_x_y
        msg_line_list = msg_line.split(',')
        # ciee_test.csv
        alt= float(msg_line_list[4])
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

imu = classOfIMU.Imu()
i = 0
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





