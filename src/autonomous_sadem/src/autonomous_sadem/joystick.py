import pygame
import time
from pygame.locals import *
import sys

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion

class Joystick:
    def __init__(self):
        # self.conn=pipe['sender']
        self.init_ros()
        pygame.init()
        pygame.display.set_caption('game base')
        self.screen = pygame.display.set_mode((1000, 500), 0, 32)
        self.clock = pygame.time.Clock()

        pygame.joystick.init()
        
        self.init_variables()
        self.joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
        for joystick in self.joysticks:
            print(joystick.get_name())

    def init_ros(self):
        rospy.init_node('joystick', anonymous=True)
        self.joystick_pub = rospy.Publisher('/joystick_command', Quaternion, queue_size=10)
        self.mode_pub = rospy.Publisher('/mode', String, queue_size=10)
        self.rate = rospy.Rate(10)

    def init_variables(self):
        self.my_square_color = 4
        self.colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255),(250,250,250),(250,250,0), (0,0,0), (255,255,255)]
        #self.motion = [0, 0]
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
        print('running',flush=True)
        self.running = True
        while True:
            self.loop()
            time.sleep(0.01)
    
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
                #print(event)
                if event.button == 0:
                    self.mode='Autonomous'
                    #print("Autonomous Mode")
                    self.my_square_color = 1
                    #self.controlU=1850
                if event.button == 3:
                    self.mode='Assisted'
                    #print("Assisted Mode")
                    self.my_square_color = 4
                    #self.controlU=1520
                if event.button == 4:
                    self.mode='Manual'
                    #print("Manual Mode")
                    self.my_square_color = 3
                    #self.controlU=1170
                # if event.button == 6:
                #     #print(pygame.joystick.Joystick.get_button(0,4))
                #     temp=self.mode
                #     self.mode='Arming...'
                #     print("Start mode")
                #     i=0
                #     self.text_surface_obj = self.font_obj.render(self.mode, True, self.colors[self.my_square_color])
                #     self.text_rect_obj1 = self.text_surface_obj.get_rect()
                #     self.text_rect_obj1.center = (500, 100)
                #     self.screen.blit(self.text_surface_obj, self.text_rect_obj1)
                #     pygame.display.update()
                #     while i<35:
                #         message="2000_1000_1000_2000_%i"%(self.controlU)
                #         print(message)
                #         #arduino.write(bytes(message, 'utf-8'))
                #         i=i+1
                #         self.clock.tick(10)
                #     self.mode=temp
            if event.type == JOYBUTTONUP:
                print(event,flush=True)
            if event.type == JOYAXISMOTION:
                #print(event)
                if event.axis ==0:
                    self.rudder=event.value*500+1500
                    self.x1=(self.rudder-1500)/4+250
                    #print("Rudder:",rudder,x1)
                elif event.axis ==1:
                    self.throttle=-event.value*500+1500
                    self.y1=-(self.throttle-1500)/4+250
                    #print("Throttle:",throttle,y1)
                #elif event.axis ==1:
                #    print(event.value)
                elif event.axis ==2:
                    self.elevator=event.value*500+1500
                    self.x2=(self.elevator-1500)/4+250+500
                    #print("Aileron:",aileron)
                    #print(event.value)
                elif event.axis ==3:
                    self.aileron=-event.value*500+1500
                    self.y2=-(self.aileron-1500)/4+250
                    #print("Elevator:",elevator)
                    #print(event.value)
                elif event.axis ==4:
                    #print(event)
                    self.l2 = event.value*500+1500
                    self.l2 = -(self.l2-1500)/4+250
                    #print("Throttle down")
                elif event.axis==5:
                    #print(event)
                    self.r2 = event.value*500+1500
                    self.r2 = -(self.r2-1500)/4+250
                    #print("Throttle up")
            if event.type == JOYHATMOTION:
                print(event,flush=True)
            if event.type == JOYDEVICEADDED:
                self.joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
                for joystick in self.joysticks:
                    print(joystick.get_name(),flush=True)
            if event.type == JOYDEVICEREMOVED:
                self.joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    pygame.quit()
                    sys.exit()

        #print("Aileron:",aileron,"Elevator",elevator,"Throttle",throttle,"Rudder",rudder)
        pygame.display.update()
        #self.output = {'commands':[self.aileron,self.elevator,self.throttle,self.rudder],'mode':self.mode}
        #print("Aileron:",self.aileron,"Elevator",self.elevator,"Throttle",self.throttle,"Rudder",self.rudder,flush=True)
        self.joystick_pub.publish(Quaternion(self.aileron,self.elevator,self.throttle,self.rudder))
        self.mode_pub.publish(self.mode)
        #self.conn.send(self.output)


if __name__ == "__main__":
    joystick = Joystick()
    joystick.run()
