# 程序一运行就开始记录小车的轨迹，以一定的时间间隔，这样来减少记录的点数
# 重点就是读取IMU数据并保存为csv
import time
import classOfIMU
import classOfDataRecord
time0 = time.time()

imu = classOfIMU.Imu()
path = r'pathRecord/'
name = '记录下的要跟踪的轨迹点' + time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())) + '.csv'
record = classOfDataRecord.DataRecord(name)

cmd_steering = 0
cmd_v = 0
i = 1
while 1:
    # 不断读取imu，然后保存为csv
    GPSWeek, GPSTime, imu_lon, imu_lat,imu_altitude, headingAngle, x, y, v, imu_gps_state, imu_satellite_num, imu_warning = imu.stateOfCar()  # 读一次
    msg = [GPSWeek, GPSTime, imu_lon, imu_lat,imu_altitude, headingAngle, x, y, v, cmd_steering, cmd_v, imu_gps_state, imu_satellite_num, imu_warning]
    # msg = [x, y, v, cmd_steering, cmd_v, imu_gps_state, imu_altitude,
    #        imu_satellite_num, imu_warning]
    record.data_record(msg)

    time.sleep(1)

del record

