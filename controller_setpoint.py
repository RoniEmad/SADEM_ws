from pyPS4Controller.controller import Controller
    
def connect():
    # any code you want to run during initial connection with the controller

        
    pass

def disconnect():
    # any code you want to run during loss of connection with the controller or keyboard interrupt
    pass    

class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
    
    def on_x_press(self):
        YAW = 90
    
    def on_L3_up(self):
        throttle += Dict['t']
        print("Throttle increased -- Throttle = ", throttle)
        
    def on_L3_down(self):
        throttle -= Dict['t']
        print("Throttle deccreased -- Throttle = ", throttle)
        
    def on_L3_left(self):
        YAW += Dict['y']
        print("YAW increased -- YAW = ", YAW)
        
    def on_L3_right(self):
        YAW -= Dict['y']
        print("YAW deccreased -- YAW = ", YAW)

    def on_R3_up(self):
        PITCH += Dict['P']
        print("PITCH increased -- PITCH = ", PITCH)
        
    def on_R3_down(self):
        PITCH -= Dict['P']
        print("PITCH deccreased -- PITCH = ", PITCH)
        
    def on_R3_left(self):
        ROLL += Dict['r']
        print("ROLL increased -- ROLL = ", ROLL)
        
    def on_R3_righ(self):
        ROLL -= Dict['r']
        print("ROLL deccreased -- ROLL = ", ROLL)
        
        
        
Dict = {'t':2, 'y':1, 'P':1, 'r':1}
throttle = 0
YAW = 0
PITCH = 0 
ROLL = 0

controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)

controller.listen(on_connect=connect, on_disconnect=disconnect)










