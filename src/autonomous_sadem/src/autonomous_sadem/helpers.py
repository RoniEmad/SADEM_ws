import math
import numpy as np

def pi_2_pi(angle):
    #A function to wrap the angle between -pi and pi (for numerical stability)
    return (angle + np.pi) % (2 * np.pi) - np.pi

class Point_class:
    def __init__(self,x:float,y:float,z:float):
        self.x=x
        self.y=y
        self.z=z

class Quaternion_class:
    def __init__(self,x:float,y:float,z:float,w:float=None): 
        #If the Inputs were (x,y,z), then we are using Euler Cooridnates. If (x,y,z,w) then the input is Quaternion  
        if w != None:
            self.x=x
            self.y=y
            self.z=z
            self.w=w
        else: #Euler to Quaternion
            self.x = math.sin(x/2) * math.cos(y/2) * math.cos(z/2) - math.cos(x/2) * math.sin(y/2) * math.sin(z/2)
            self.y = math.cos(x/2) * math.sin(y/2) * math.cos(z/2) + math.sin(x/2) * math.cos(y/2) * math.sin(z/2)
            self.z = math.cos(x/2) * math.cos(y/2) * math.sin(z/2) - math.sin(x/2) * math.sin(y/2) * math.cos(z/2)
            self.w = math.cos(x/2) * math.cos(y/2) * math.cos(z/2) + math.sin(x/2) * math.sin(y/2) * math.sin(z/2)
        #print(self.x, self.y, self.z, self.w)
    def quaternion(self):
        return np.array([self.x, self.y, self.z, self.w])
    def euler_zyx(self):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        x=self.x
        y=self.y
        z=self.z
        w=self.w

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return np.array([roll_x, pitch_y, yaw_z]) #In radians

class Pose_class:
    def __init__(self,position:Point_class=None, orientation:Quaternion_class=None):
        self.position=position
        self.orientation=orientation
    def get_position(self):
        return np.array([self.position.x, self.position.y, self.position.z])
    def get_orientation(self, type:str='euler'):
        if type=='quaternion':
            return self.orientation.quaternion()
        elif type=='euler':
            return self.orientation.euler_zyx()
        else:
            raise ValueError('type must be either euler or quaternion')