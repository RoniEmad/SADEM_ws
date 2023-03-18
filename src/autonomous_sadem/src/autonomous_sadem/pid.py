import math
from simple_pid import PID
from autonomous_sadem.helpers import pi_2_pi

class PID_Controller:
    def __init__(self):
        self.z_pid = PID(2, 1, 0, setpoint=0)
        self.y_pid = PID(1.5, 0.004, 4, setpoint=0)
        self.x_pid = PID(1.5, 0.004, 4, setpoint=0)
        self.yaw_pid = PID(0.1, 0.01, 0.1, setpoint=0)
        
        self.max_diffs = {
            'x': 0.1,
            'y': 0.1,
            'z': 0.1,
            'yaw': math.pi/12
        }

        self.x_pid.output_limits = (-0.05, 0.05)       #maximum drone pitch angle
        self.y_pid.output_limits = (-0.05, 0.05)       #maximum drone roll angle
        self.yaw_pid.output_limits = (-0.05, 0.05)     #maximum drone yaw angle

    def update(self, goal, state):
        print("control update")
        print("Goal: ", goal.get_position(),goal.get_orientation())
        print("State: ", state.get_position(),state.get_orientation())

        #Yaw angle Controller
        phi= pi_2_pi(goal.orientation.euler_zyx()[2] - state.orientation.euler_zyx()[2]) #In radians
        
        if phi > self.max_diffs['yaw']:
            phi = self.max_diffs['yaw']
        elif phi < -self.max_diffs['yaw']:
            phi = -self.max_diffs['yaw']

        yaw_ = -self.yaw_pid(phi)

        #x,y position Controller
        theta=state.orientation.euler_zyx()[2]                  # Drone Orientation
        delta = goal.get_position()-state.get_position()        # Difference in positions in x,y,z between goal point and current state point
                                                                # with reference to the global frame
        x_=delta[0]*math.cos(theta)+delta[1]*math.sin(theta)    # Get the difference in x with reference to the drone frame
        y_=-delta[0]*math.sin(theta)+delta[1]*math.cos(theta)   # Get the difference in y with reference to the drone frame
        z_=delta[2]
        if x_ > self.max_diffs['x']:
            x_ = self.max_diffs['x']
        elif x_ < -self.max_diffs['x']:
            x_ = -self.max_diffs['x']
        if y_ > self.max_diffs['y']:
            y_ = self.max_diffs['y']
        elif y_ < -self.max_diffs['y']:
            y_ = -self.max_diffs['y']
        if z_ > self.max_diffs['z']:
            z_ = self.max_diffs['z']
        elif z_ < -self.max_diffs['z']:
            z_ = -self.max_diffs['z']

        aierlon_ = self.y_pid(y_)
        rudder_ = self.x_pid(x_)
        throttle_ = self.z_pid(z_)
        
        return self.pid_to_ppm([aierlon_, rudder_, throttle_, yaw_])
    
    def pid_to_ppm(self, commands):
        # aileron, rudder, throttle, yaw are in range [-1, 1]
        # return aileron, rudder, throttle, yaw in range [1000, 2000]
        for i in range(len(commands)):
            if commands[i] > 1:
                commands[i] = 1
            elif commands[i] < -1:
                commands[i] = -1
            commands[i] = commands[i] * 500 + 1500
        return commands