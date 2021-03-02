import rospy
import rosbag
import sys
import math
from sensor_msgs.msg import Imu

if sys.getdefaultencoding() != 'utf-8':
    reload(sys)
    sys.setdefaultencoding('utf-8')
bag_name = '2020-10-17-09-09-38.bag' #被修改的bag名
out_bag_name = 'out_2020-10-17-09-09-38.bag' #修改后的bag名
IMU_name='rawimu.txt'#IMU名
att_name='attpos.txt'
dst_dir = '/home/feiyanghe/cal-1018/02/' #使用路径

f0=open(dst_dir+att_name,'r')
f1=open(dst_dir+IMU_name,'r')


def EulerAndQuaternionTransform( intput_data):
	data_len = len(intput_data)
	angle_is_not_rad = True
 
	if data_len == 3:
		r = 0
		p = 0
		y = 0
		if angle_is_not_rad: # 180 ->pi
			r = math.radians(intput_data[0]) 
			p = math.radians(intput_data[1])
			y = math.radians(intput_data[2])
		else:
			r = intput_data[0] 
			p = intput_data[1]
			y = intput_data[2]
 
		sinp = math.sin(p/2)
		siny = math.sin(y/2)
		sinr = math.sin(r/2)
 
		cosp = math.cos(p/2)
		cosy = math.cos(y/2)
		cosr = math.cos(r/2)
 
		w = cosr*cosp*cosy + sinr*sinp*siny
		x = sinr*cosp*cosy - cosr*sinp*siny
		y = cosr*sinp*cosy + sinr*cosp*siny
		z = cosr*cosp*siny - sinr*sinp*cosy
 
		return w,x,y,z
 
	elif data_len == 4:
 
		w = intput_data[0] 
		x = intput_data[1]
		y = intput_data[2]
		z = intput_data[2]
 
		r = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
		p = math.asin(2 * (w * y - z * x))
		y = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
 
		if angle_is_not_rad ==False: # 180 ->pi
 
			r = r / math.pi * 180
			p = p / math.pi * 180
			y = y / math.pi * 180
 
		return {r,p,y}



with rosbag.Bag(dst_dir+out_bag_name, 'w') as outbag:
    #stamp = None
    #topic:就是发布的topic msg:该topic在当前时间点下的message t:消息记录时间(非header)
    ##read_messages内可以指定的某个topic
    s0=f0.readline()
    while s0:
        sli0=s0.split(",")
        s0=f0.readline()
        if float(sli0[0])<1602897022 or float(sli0[0])>1602897070:
            continue
        strline=f1.readline()
        while strline:
            sli=strline.split(",")
            strline=f1.readline()        
            if float(sli[0])<float(sli0[0]):
                continue
            if float(sli[0])==float(sli0[0]):
                print(sli[0])

                w,x,y,z=EulerAndQuaternionTransform([float(sli0[4]),float(sli0[5]),float(sli0[6])])

                m_imu=Imu()
                m_imu.header.stamp = rospy.Time.from_sec(float(sli[0]))
                m_imu.linear_acceleration.x=float(sli[4])
                m_imu.linear_acceleration.y=float(sli[5])
                m_imu.linear_acceleration.z=float(sli[6])
                m_imu.angular_velocity.x=float(sli[1])
                m_imu.angular_velocity.y=float(sli[2])
                m_imu.angular_velocity.z=float(sli[3])
                m_imu.orientation.w=w
                m_imu.orientation.x=x
                m_imu.orientation.y=y
                m_imu.orientation.z=z
                outbag.write('/imu', m_imu, m_imu.header.stamp)
                break

    for topic, msg, t in rosbag.Bag(dst_dir+bag_name).read_messages():
        if topic == '/velodyne_points':
            stamp = msg.header.stamp
            #stamp.secs=stamp.secs-734
            if stamp.secs<1602897022 or stamp.secs>1602897070:
                continue
            outbag.write(topic, msg, stamp)
            continue
        else:
            stamp = msg.header.stamp
            #if stamp.secs<1608627400:
                #stamp.secs=stamp.secs+3600
            if stamp.secs<1602897022 or stamp.secs>1602897070:
                continue
            outbag.write(topic, msg, stamp)
            continue
    


print("finished")