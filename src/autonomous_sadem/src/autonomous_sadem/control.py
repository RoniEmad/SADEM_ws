from autonomous_sadem.helpers import Point_class, Quaternion_class, Pose_class
from autonomous_sadem.pid import PID_Controller

from autonomous_sadem.ppm import PPM
import pigpio

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import String

class Controller:
    def __init__(self):
        self.pid_controller = PID_Controller()
        self.init_ros()
        self.goal=Pose_class(Point_class(0,0,0),Quaternion_class(0,0,0,1))
        self.state=Pose_class(Point_class(0,0,0),Quaternion_class(0,0,0,1))
        self.initPPM()
        self.mode="Assisted"
        self.commands=[1500,1500,1500,1500]
        self.extra_commands=[1500,1500,1700,1000]
        self.control_loop()


    def init_ros(self):
        rospy.init_node('control_node', anonymous=True)
        self.goal_subscriber = rospy.Subscriber("/goal_pose", PoseStamped, self.set_goal_pose)
        self.state_subscriber = rospy.Subscriber("/current_pose", PoseStamped, self.state_receiver)
        self.joystick_subscriber = rospy.Subscriber("/joystick_command", Quaternion, self.joystick_receiver)
        self.joystick_extra_subscriber = rospy.Subscriber("/joystick_extra_command", Quaternion, self.joystick_extra_receiver)
        self.mode_subscriber = rospy.Subscriber("/mode", String, self.mode_receiver)
        self.ppm_pub = rospy.Publisher('/ppm', Quaternion, queue_size=10)
        #self.rate = rospy.Rate(100)
    
    def initPPM(self):
        
        self.pi = pigpio.pi()

        if not self.pi.connected:
            exit(0)

        self.pi.wave_tx_stop() # Start with a clean slate.

        self.ppm = PPM(self.pi, 6, frame_ms=20)

    def set_goal_pose(self,goal_msg): # Pose msg
        """
            This is the callback of a ros subscriber,
            It receives the Goal Pose and store it in a global variable (goal_point)
        """
        goal_msg=goal_msg.pose
        print(goal_msg)
        x,y,z,w=goal_msg.orientation.x,goal_msg.orientation.y,goal_msg.orientation.z,goal_msg.orientation.w
        if self.mode == "Autonomous":
            self.goal = Pose_class(goal_msg.position,Quaternion_class(x,y,z,w))
        
    def state_receiver(self,current_msg):
        """
            This is the callback of a ros subscriber,
            It receives the Current Drone Pose and store it in a global variable (current_state)
            Then do one iteration in the control loop
        """
        current_msg=current_msg.pose
        x,y,z,w=current_msg.orientation.x,current_msg.orientation.y,current_msg.orientation.z,current_msg.orientation.w
        self.state = Pose_class(current_msg.position,Quaternion_class(x,y,z,w))
    
    def joystick_receiver(self,joystick_msg):
        """
            This is the callback of a ros subscriber,
            It receives the Joystick commands and store it in a global variable (joystick_commands)
        """
        self.commands=[joystick_msg.x,joystick_msg.y,joystick_msg.z,joystick_msg.w]
    
    def joystick_extra_receiver(self,joystick_msg):
        """
            This is the callback of a ros subscriber,
            It receives the Joystick commands and store it in a global variable (joystick_commands)
        """
        #print("extra",joystick_msg)
        self.extra_commands=[joystick_msg.x,joystick_msg.y,joystick_msg.z,joystick_msg.w]

    def mode_receiver(self,mode_msg):
        """
            This is the callback of a ros subscriber,
            It receives the Mode and store it in a global variable (mode)
        """
        self.mode=mode_msg.data

    def update_control(self):
        if self.mode == "Manual":
            self.manual_mode()
        elif self.mode == "Assisted":
            self.assisted_mode()
        elif self.mode == "Autonomous":
            self.autonomous_mode()

    def manual_mode(self):
        print("Manual mode",self.commands, flush=True)
        self.sendPPM(self.commands+self.extra_commands)
        
    def assisted_mode(self):
        print("Assisted mode",self.commands, flush=True)

        y,x,z,yaw=self.commands

        #print("self.state",self.state.get_position(),self.state.get_orientation(), flush=True)
        #print("self.goal",self.goal.get_position(),self.goal.get_orientation(), flush=True)
        
        new_x,new_y,new_z,new_yaw=self.state.get_position()[0],self.state.get_position()[1],self.state.get_position()[2],self.state.get_orientation()[2]
        changed=False
        if x < 1200:
            new_x = new_x-0.1
            changed=True
        elif x > 1800:
            new_x = new_x+0.1
            changed=True
        if y < 1200:
            new_y = new_y-0.1
            changed=True
        elif y > 1800:
            new_y = new_y+0.1
            changed=True
        if z < 1200:
            new_z = new_z+0.1
            changed=True
        elif z > 1800:
            new_z = new_z-0.1
            changed=True
        if yaw < 1200:
            new_yaw = new_yaw-0.1
            changed=True
        elif yaw > 1800:
            new_yaw = new_yaw+0.1
            changed=True
        if changed:
            self.goal=Pose_class(Point_class(new_x,new_y,new_z),Quaternion_class(0,0,new_yaw))
        
        self.control_loop()

    def autonomous_mode(self):
        print("Autonomous mode", flush=True)
        self.control_loop()

    def sendPPM(self,msg:list,all=False):
        #print('PPM msg: ',msg, flush=True)
        self.ppm_pub.publish(Quaternion(msg[0],msg[1],msg[2],msg[3]))
        if all:
            print('PPM all channels: ',msg, flush=True)
            #pass
            self.ppm.update_channels(msg)
        else:
            for i in range(len(msg)):
                if msg[i] != 0:
                    #print('PPM channel: ',i,' value: ',int(msg[i]), flush=True)
                    self.ppm.update_channel(i, int(msg[i]))
                    #pass

    def control_loop(self):
        #print(self.pid_controller.update(self.goal,self.state),'extra',self.extra_commands)
        self.sendPPM(self.pid_controller.update(self.goal,self.state)+ self.extra_commands)

    def close(self):
        self.ppm.cancel()
        self.pi.stop()

if __name__ == "__main__":
    controller=Controller()
    while not rospy.is_shutdown():
        controller.update_control()
        #controller.rate.sleep()

    controller.close()
