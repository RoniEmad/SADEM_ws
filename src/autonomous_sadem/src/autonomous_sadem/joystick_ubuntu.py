import pygame
import time
from pygame.locals import *
import sys

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion

class Bindings:
    def __init__(self):
        self.ch1 = 1500
        self.ch2 = 1500
        self.ch3 = 1500
        self.ch4 = 1500
        self.ch5 = 1500
        self.ch6 = 1500
        self.ch7 = 1700
        self.ch8 = 1000
    def get_controls(self):
        return [self.ch1,self.ch2,self.ch3,self.ch4]
    def get_extra_controls(self):
        return [self.ch5,self.ch6,self.ch7,self.ch8]

class Joystick:
    def __init__(self):
        self.bindings=Bindings()
        self.init_ros()
        pygame.init()
        pygame.display.set_caption('game base')
        self.screen = pygame.display.set_mode((1000, 500), 0, 32)
        self.clock = pygame.time.Clock()

        pygame.joystick.init()
        
        self.init_variables()
        #self.joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
        self.joysticks=[pygame.joystick.Joystick(0)]
        for joystick in self.joysticks:
            print(joystick.get_name())

    def init_ros(self):
        rospy.init_node('joystick', anonymous=True)
        self.joystick_pub = rospy.Publisher('/joystick_command', Quaternion, queue_size=10)
        self.joystick_extra_pub = rospy.Publisher('/joystick_extra_command', Quaternion, queue_size=10)
        self.mode_pub = rospy.Publisher('/mode', String, queue_size=10)
        
    def init_variables(self):
        self.my_square_color = 4
        self.colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255),(250,250,250),(250,250,0), (0,0,0), (255,255,255)]
        self.throttle=1500
        self.rudder=1500
        self.elevator=1500
        self.aileron=1500
        self.controlU=1520
        self.x1=250
        self.y1=250
        self.x2=250+500
        self.y2=250
        self.l2=250+125
        self.r2=250+125
        self.mode='Assisted'

    def run(self):
        #print('running',flush=True)
        self.running = True
        while True:
            self.loop()
            #time.sleep(0.01)
    
    def update(self):
        self.running = True
        self.loop()

    def stop(self):
        self.running = False

    def update_screen(self):
        self.screen.fill((0, 0, 0))

        self.font_obj = pygame.font.Font('freesansbold.ttf', 32)
        self.text_surface_obj = self.font_obj.render(self.mode, True, self.colors[self.my_square_color])
        self.text_rect_obj = self.text_surface_obj.get_rect()
        self.text_rect_obj.center = (500, 50)
        self.screen.blit(self.text_surface_obj, self.text_rect_obj)
        
        
        pygame.draw.circle(self.screen, (200,200,200), [250,250],175,0)
        pygame.draw.circle(self.screen, self.colors[1], [self.x1,self.y1],10)
        pygame.draw.line(self.screen, (0,250,0), (75, 125), (425, 125), 4)
        pygame.draw.line(self.screen, (0,250,0), (75, 500-125), (425, 500-125), 4)
        pygame.draw.line(self.screen, (0,250,0), (125, 75), (125, 425), 4)
        pygame.draw.line(self.screen, (0,250,0), (500-125, 75), (500-125, 425), 4)

        pygame.draw.line(self.screen, (250,0,0), (200, 250), (300, 250), 2)
        pygame.draw.line(self.screen, (250,0,0), (250, 200), (250, 300), 2)

        pygame.draw.circle(self.screen, (200,200,200), [250+500,250],175,0)
        pygame.draw.circle(self.screen, self.colors[2], [self.x2,self.y2],10)
        pygame.draw.line(self.screen, (0,250,0), (75+500, 125), (425+500, 125), 4)
        pygame.draw.line(self.screen, (0,250,0), (75+500, 500-125), (425+500, 500-125), 4)
        pygame.draw.line(self.screen, (0,250,0), (125+500, 75), (125+500, 425), 4)
        pygame.draw.line(self.screen, (0,250,0), (500-125+500, 75), (500-125+500, 425), 4)

        pygame.draw.line(self.screen, (250,0,0), (200+500, 250), (300+500, 250), 2)
        pygame.draw.line(self.screen, (250,0,0), (250+500, 200), (250+500, 300), 2)

        pygame.draw.line(self.screen, (100,100,100), (40, 380), (40, 120), 15)
        pygame.draw.line(self.screen, (100,100,100), (250+710, 380), (250+710, 120), 15)
        pygame.draw.circle(self.screen, self.colors[3], [40,self.l2],10)
        pygame.draw.circle(self.screen, self.colors[4], [250+710,self.r2],10)



    def loop(self):
        self.update_screen()
        for event in pygame.event.get():
            if event.type == JOYBUTTONDOWN:
                print(event.button)
                if event.button == 0:
                    self.mode='Autonomous'
                    self.my_square_color = 1
                if event.button == 2:
                    self.mode='Assisted'
                    self.my_square_color = 4
                if event.button == 8:
                    self.mode='Manual'
                    self.my_square_color = 3
                if event.button == 9:
                    self.mode_old=self.mode
                    self.color_old=self.my_square_color
                    self.mode='Arming/Disarming'
                    self.my_square_color = 2
                if event.button == 1:
                    self.bindings.ch7 = 1800
                if event.button == 3:
                    self.bindings.ch7 = 1600
                if event.button == 4:
                    self.bindings.ch7 = 1700
            if event.type == JOYBUTTONUP:
                if event.button == 9:
                    self.mode=self.mode_old
                    self.my_square_color=self.color_old
                #print(event,flush=True)
            if event.type == JOYAXISMOTION:
                if event.axis ==0:
                    self.rudder=event.value*500+1500
                    self.x1=(self.rudder-1500)/4+250
                    #if self.rudder>1800:
                    #    self.rudder=2000
                    #elif self.rudder<1200:
                    #    self.rudder=1000
                    #else:
                    self.rudder-=1500
                    self.rudder*=0.5
                    self.rudder+=1500
                    #self.bindings.ch4=self.rudder
                elif event.axis ==1:
                    self.throttle=-event.value*500+1500
                    self.y1=-(self.throttle-1500)/4+250
                    #if self.throttle>1800:
                    #    self.throttle=2000
                    #el
                    self.throttle-=1500
                    self.throttle*=0.55
                    self.throttle+=1275
                    #self.throttle-=100
                    #if self.throttle<1000:
                    #    self.throttle=1000

                    #self.bindings.ch3=self.throttle
                elif event.axis ==3:
                    self.aileron=event.value*500+1500
                    self.x2=(self.aileron-1500)/4+250+500
                    #if self.aileron>1800:
                    #    self.aileron=2000
                    #elif self.aileron<1200:
                    #    self.aileron=1000
                    #else:
                    self.aileron-=1500
                    self.aileron*=0.5
                    self.aileron+=1500
                    
                    #self.bindings.ch1=self.aileron
                elif event.axis ==4:
                    self.elevator=-event.value*500+1500
                    self.y2=-(self.elevator-1500)/4+250
                    #if self.elevator>1800:
                    #    self.elevator=2000
                    #elif self.elevator<1200:
                    #    self.elevator=1000
                    #else:
                    self.elevator-=1500
                    self.elevator*=-0.5
                    self.elevator+=1500

                    #self.bindings.ch2=self.elevator
                elif event.axis ==2:
                    self.l2_ = event.value*500+1500
                    self.l2 = -(self.l2_-1500)/4+250

                    #self.bindings.ch7=self.l2_
                elif event.axis==5:
                    self.r2_ = event.value*500+1500
                    self.r2 = -(self.r2_-1500)/4+250

                    self.bindings.ch6=self.r2_
            if event.type == JOYHATMOTION:
                print(event,flush=True)
            if event.type == JOYDEVICEADDED:
                #self.joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
                self.joysticks=[pygame.joystick.Joystick(0)]
                for joystick in self.joysticks:
                    print(joystick.get_name(),flush=True)
            if event.type == JOYDEVICEREMOVED:
                self.joysticks = self.joysticks=[pygame.joystick.Joystick(0)]
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    pygame.quit()
                    sys.exit()
        pygame.display.update()
        if self.mode=="Arming/Disarming":
            #print("arming")
            self.bindings.ch1=1000
            self.bindings.ch2=1000
            self.bindings.ch3=1000
            self.bindings.ch4=2000
            self.mode_to_pub="Manual"
        else:
            self.bindings.ch1=self.aileron
            self.bindings.ch2=self.elevator
            self.bindings.ch3=self.throttle
            self.bindings.ch4=self.rudder
            self.mode_to_pub=self.mode
        #print("dsad",self.bindings.get_controls())
        ch1,ch2,ch3,ch4= self.bindings.get_controls()
        ch5,ch6,ch7,ch8= self.bindings.get_extra_controls()
        self.joystick_pub.publish(Quaternion(ch1,ch2,ch3,ch4))
        self.joystick_extra_pub.publish(Quaternion(ch5,ch6,ch7,ch8))
        self.mode_pub.publish(self.mode_to_pub)

if __name__ == "__main__":
    joystick = Joystick()
    joystick.run()
