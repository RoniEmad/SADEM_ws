#!/usr/bin/env python3
import time
import roslib; roslib.load_manifest('roscopter')
import rospy
from std_msgs.msg import String, Header
from std_srvs.srv import *
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
import roscopter.msg
import sys,struct,time,os
from nav_msgs.msg import Odometry
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '../mavlink/pymavlink'))
import tf
from geometry_msgs.msg import PoseStamped

from optparse import OptionParser
parser = OptionParser("roscopter.py [options]")

parser.add_option("--baudrate", dest="baudrate", type='int',
                  help="master port baud rate", default=115200)
parser.add_option("--device", dest="device", default="/dev/ttyACM0", help="serial device")
parser.add_option("--rate", dest="rate", default=100, type='int', help="requested stream rate")
parser.add_option("--source-system", dest='SOURCE_SYSTEM', type='int',
                  default=255, help='MAVLink source system for this GCS')
parser.add_option("--enable-control",dest="enable_control", default=True, help="Enable listning to control messages")

(opts, args) = parser.parse_args()
rospy.init_node("imu_pub")
odom_pub = rospy.Publisher('/current_pose', PoseStamped, queue_size=1)
import mavutil

zed_pos = [0,0,0]
def zed_callback(data):
    global zed_pos
    zed_pos[0] = data.pose.pose.position.x
    zed_pos[1] = data.pose.pose.position.y
    zed_pos[2] = data.pose.pose.position.z
rospy.Subscriber('/zed/slam', Odometry, zed_callback)
# create a mavlink serial instance
master = mavutil.mavlink_connection(opts.device, baud=opts.baudrate)
if opts.device is None:
    print("You must specify a serial device")
    sys.exit(1)

def wait_heartbeat(m):
    '''wait for a heartbeat so we know the target system IDs'''
    print("Waiting for APM heartbeat")
    m.wait_heartbeat()
    print("Heartbeat from APM (system %u component %u)" % (m.target_system, m.target_system))

def send_rc(data):
    master.mav.rc_channels_override_send(master.target_system, master.target_component,data.channel[0],data.channel[1],data.channel[2],data.channel[3],data.channel[4],data.channel[5],data.channel[6],data.channel[7])
    print ("sending rc: %s"%data)


def mainloop():
    while True:
        msg = master.recv_match(blocking=False)
        if not msg:
            continue

        msg_type = msg.get_type()
        if msg_type == "BAD_DATA":
            if mavutil.all_printable(msg.data):
                sys.stdout.write(str(msg.data))
                sys.stdout.flush()
            continue
        
        data = None
        if msg_type == "RC_CHANNELS_RAW" :
            data = [msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw, msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw]
        if msg_type == "VFR_HUD":
            data = [msg.airspeed, msg.groundspeed, msg.heading, msg.throttle, msg.alt, msg.climb]
        if msg_type == "ATTITUDE" :
            data = [msg.roll, msg.pitch, msg.yaw, msg.rollspeed, msg.pitchspeed, msg.yawspeed]
        if msg_type == "RAW_IMU" :
            data = [msg.time_usec, 
                        msg.xacc, msg.yacc, msg.zacc, 
                        msg.xgyro, msg.ygyro, msg.zgyro,
                        msg.xmag, msg.ymag, msg.zmag]
            msg = PoseStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = rospy.Time.now()
            quat = tf.transformations.quaternion_from_euler(data[-3], data[-2], data[-1])
            msg.pose.orientation.x = quat[0]
            msg.pose.orientation.y = quat[1]
            msg.pose.orientation.z = quat[2]
            msg.pose.orientation.w = quat[3]
            msg.pose.position.x = zed_pos[0]
            msg.pose.position.y = zed_pos[2]
            msg.pose.position.z = -1*zed_pos[1]
            odom_pub.publish(msg)

           

        if data is not None:
            to_send = {msg_type:data}
            # send over udp
            print(to_send)
        #time.sleep(0.0001)



# wait for the heartbeat msg to find the system ID
wait_heartbeat(master)


# waiting for 10 seconds for the system to be ready
print("Sleeping for 10 seconds to allow system, to be ready")
rospy.sleep(10)
print("Sending all stream request for rate %u" % opts.rate)
#for i in range(0, 3):
#print(master.target_component,mavutil.mavlink.MAV_DATA_STREAM_ALL,"this",master.DATA_STREAM)
print(master.mav)
master.mav.request_data_stream_send(master.target_system, master.target_component,
                                    mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, opts.rate, 1)

#master.mav.set_mode_send(master.target_system, 
if __name__ == '__main__':
    try:
        mainloop()
    except rospy.ROSInterruptException: pass

