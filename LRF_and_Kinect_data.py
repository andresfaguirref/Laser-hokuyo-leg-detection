## -- Libraries  -- Libraries  -- Libraries  -- Libraries  -- Libraries  -- Libraries   

import serial
import hokuyo
import serial_port
import time
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.animation as animation
import threading


from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime

import ctypes
import _ctypes
import pygame
import sys

import time

import xlwt

if sys.hexversion >= 0x03000000:
    import _thread as thread
else:
    import thread

## -- Laser range finder scan object -- Laser range finder scan object -- Laser range finder scan object

class LRF(object):
    def __init__(self,uart_speed = 9800,uart_port = "COM1",star_step = 44,stop_step = 725):
      
        self.laser_serial = serial.Serial(port=uart_port, baudrate=uart_speed, timeout=0.5)
        self.port = serial_port.SerialPort(self.laser_serial)
        self.laser = hokuyo.Hokuyo(self.port)
        self.star_step = star_step
        self.stop_step = stop_step

    def laser_on(self):
        self.laser.laser_on()

    def laser_off(self):
        self.laser.laser_off()
        
    def read_one_scan(self):
        return self.laser.get_single_scan(self.star_step, self.stop_step, 1)

## -- Port LRF detection -- Port LRF detection -- Port LRF detection -- Port LRF detection 

def Find_port_URG_laser():
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())
    Found = False 
    for x in ports:
        if x[1].find("URG Series") == 0:
            Laser_port = x[0]
            Found = True
            break

    if Found == True:
        return Laser_port
    else:
        print ("The LRF hokuyo wasn't found")
        return "Error"

## -- Animate plot fuction -- Animate plot fuction -- Animate plot fuction -- Animate plot fuction
    
def animate(i):

    global Laser_lec
    
    theta = np.array([x[0]*np.pi/180 for x in Laser_lec])
    radius = np.array([x[1] for x in Laser_lec])
    
    ax.clear()
    ax.plot(theta, radius, color='b', linewidth=1)
    
    ax.set_rmax(2000)
    ax.grid(True)
    plt.title("LRF scan", va='bottom')

## -- Thread for LRF Lecture

def LRF_Lecture():
    
    global Laser_lecture,Laser_obj,Laser_lec,Time_LRF

    while Laser_lecture:
        Laser_lec = Laser_obj.read_one_scan()
        Time_LRF.append(time.time())
        #Laser_lec1 = []
        #for x in range (0,len(Laser_lec)):
            #if Laser_lec[x][1] > 2000:
                #Laser_lec1.append((Laser_lec[x][0],0))
            #else:
                #Laser_lec1.append(Laser_lec[x])
        #Laser_lec = Laser_lec1
                
        Laser_scan_regitry.append(Laser_lec)

    print("Laser off")

## -- Kinect object -- Kinect object -- Kinect object -- Kinect object -- Kinect object

# colors for drawing different bodies 
SKELETON_COLORS = [pygame.color.THECOLORS["red"], 
                  pygame.color.THECOLORS["blue"], 
                  pygame.color.THECOLORS["green"], 
                  pygame.color.THECOLORS["orange"], 
                  pygame.color.THECOLORS["purple"], 
                  pygame.color.THECOLORS["yellow"], 
                  pygame.color.THECOLORS["violet"]]


class BodyGameRuntime(object):
    def __init__(self):
        pygame.init()

        # Used to manage how fast the screen updates
        self._clock = pygame.time.Clock()

        # Set the width and height of the screen [width, height]
        self._infoObject = pygame.display.Info()
        self._screen = pygame.display.set_mode((self._infoObject.current_w >> 1, self._infoObject.current_h >> 1), 
                                               pygame.HWSURFACE|pygame.DOUBLEBUF|pygame.RESIZABLE, 32)

        pygame.display.set_caption("Kinect for Windows v2 Body Game")

        # Loop until the user clicks the close button.
        self._done = False

        # Used to manage how fast the screen updates
        self._clock = pygame.time.Clock()

        # Kinect runtime object, we want only color and body frames 
        self._kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Body)

        # back buffer surface for getting Kinect color frames, 32bit color, width and height equal to the Kinect color frame size
        self._frame_surface = pygame.Surface((self._kinect.color_frame_desc.Width, self._kinect.color_frame_desc.Height), 0, 32)

        # here we will store skeleton data 
        self._bodies = None


    def draw_body_bone(self, joints, jointPoints, color, joint0, joint1):
        joint0State = joints[joint0].TrackingState;
        joint1State = joints[joint1].TrackingState;

        # both joints are not tracked
        if (joint0State == PyKinectV2.TrackingState_NotTracked) or (joint1State == PyKinectV2.TrackingState_NotTracked): 
            return

        # both joints are not *really* tracked
        if (joint0State == PyKinectV2.TrackingState_Inferred) and (joint1State == PyKinectV2.TrackingState_Inferred):
            return

        # ok, at least one is good 
        start = (jointPoints[joint0].x, jointPoints[joint0].y)
        end = (jointPoints[joint1].x, jointPoints[joint1].y)

        try:
            pygame.draw.line(self._frame_surface, color, start, end, 8)
        except: # need to catch it due to possible invalid positions (with inf)
            pass

    def draw_body(self, joints, jointPoints, color):
        # Torso
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_Head, PyKinectV2.JointType_Neck);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_Neck, PyKinectV2.JointType_SpineShoulder);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_SpineMid);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineMid, PyKinectV2.JointType_SpineBase);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_ShoulderRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_ShoulderLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipLeft);
    
        # Right Arm    
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ShoulderRight, PyKinectV2.JointType_ElbowRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ElbowRight, PyKinectV2.JointType_WristRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristRight, PyKinectV2.JointType_HandRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HandRight, PyKinectV2.JointType_HandTipRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristRight, PyKinectV2.JointType_ThumbRight);

        # Left Arm
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ShoulderLeft, PyKinectV2.JointType_ElbowLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ElbowLeft, PyKinectV2.JointType_WristLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_HandLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HandLeft, PyKinectV2.JointType_HandTipLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_ThumbLeft);

        # Right Leg
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HipRight, PyKinectV2.JointType_KneeRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_KneeRight, PyKinectV2.JointType_AnkleRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_AnkleRight, PyKinectV2.JointType_FootRight);

        # Left Leg
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HipLeft, PyKinectV2.JointType_KneeLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_KneeLeft, PyKinectV2.JointType_AnkleLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_AnkleLeft, PyKinectV2.JointType_FootLeft);


    def draw_color_frame(self, frame, target_surface):
        target_surface.lock()
        address = self._kinect.surface_as_array(target_surface.get_buffer())
        ctypes.memmove(address, frame.ctypes.data, frame.size)
        del address
        target_surface.unlock()

    def run(self):
        # -------- Main Program Loop -----------
        while not self._done:
            # --- Main event loop
            for event in pygame.event.get(): # User did something
                if event.type == pygame.QUIT: # If user clicked close
                    self._done = True # Flag that we are done so we exit this loop

                elif event.type == pygame.VIDEORESIZE: # window resized
                    self._screen = pygame.display.set_mode(event.dict['size'], 
                                               pygame.HWSURFACE|pygame.DOUBLEBUF|pygame.RESIZABLE, 32)
                    
            # --- Game logic should go here

            # --- Getting frames and drawing  
            # --- Woohoo! We've got a color frame! Let's fill out back buffer surface with frame's data 
            if self._kinect.has_new_color_frame():
                frame = self._kinect.get_last_color_frame()
                self.draw_color_frame(frame, self._frame_surface)
                frame = None

            # --- Cool! We have a body frame, so can get skeletons
            if self._kinect.has_new_body_frame(): 
                self._bodies = self._kinect.get_last_body_frame()

            # --- draw skeletons to _frame_surface
            if self._bodies is not None: 
                for i in range(0, self._kinect.max_body_count):
                    body = self._bodies.bodies[i]
                    if not body.is_tracked: 
                        continue
                    
                    joints = body.joints

                    
                    # convert joint coordinates to color space 
                    joint_points = self._kinect.body_joints_to_color_space(joints)
                    self.draw_body(joints, joint_points, SKELETON_COLORS[i])

# values for enumeration '_JointType'
# JointType_SpineBase = 0
# JointType_SpineMid = 1
# JointType_Neck = 2
# JointType_Head = 3
# JointType_ShoulderLeft = 4
# JointType_ElbowLeft = 5
# JointType_WristLeft = 6
# JointType_HandLeft = 7
# JointType_ShoulderRight = 8
# JointType_ElbowRight = 9
# JointType_WristRight = 10
# JointType_HandRight = 11
# JointType_HipLeft = 12
# JointType_KneeLeft = 13
# JointType_AnkleLeft = 14
# JointType_FootLeft = 15
# JointType_HipRight = 16
# JointType_KneeRight = 17
# JointType_AnkleRight = 18
# JointType_FootRight = 19
# JointType_SpineShoulder = 20
# JointType_HandTipLeft = 21
# JointType_ThumbLeft = 22
# JointType_HandTipRight = 23
# JointType_ThumbRight = 24

                    if True:
                        data = []
                        for VAR1 in range (0,25):
                            X = joints[VAR1].Position.x
                            Y = joints[VAR1].Position.y
                            Z = joints[VAR1].Position.z

                            Q1 = body.joint_orientations[VAR1].Orientation.w
                            Q2 = body.joint_orientations[VAR1].Orientation.x
                            Q3 = body.joint_orientations[VAR1].Orientation.y
                            Q4 = body.joint_orientations[VAR1].Orientation.z

                            data.append([X, Y, Z, Q1, Q2, Q3, Q4])

                        global Kinect_data,Time_kinect 
                        Kinect_data.append(data)
                        Time_kinect.append(time.time())
            else:
                data = []
                for VAR1 in range (0,25):
                    X = 0
                    Y = 0
                    Z = 0

                    Q1 = 0
                    Q2 = 0
                    Q3 = 0
                    Q4 = 0

                    data.append([X, Y, Z, Q1, Q2, Q3, Q4])

                global Kinect_data,Time_kinect 
                Kinect_data.append(data)
                Time_kinect.append(time.time())
                                                   

            # --- copy back buffer surface pixels to the screen, resize it if needed and keep aspect ratio
            # --- (screen size may be different from Kinect's color frame size) 
            h_to_w = float(self._frame_surface.get_height()) / self._frame_surface.get_width()
            target_height = int(h_to_w * self._screen.get_width())
            surface_to_draw = pygame.transform.scale(self._frame_surface, (self._screen.get_width(), target_height));
            self._screen.blit(surface_to_draw, (0,0))
            surface_to_draw = None
            pygame.display.update()

            # --- Go ahead and update the screen with what we've drawn.
            pygame.display.flip()

            # --- Limit to 60 frames per second
            self._clock.tick(30)

        # Close our Kinect sensor, close the window and quit.
        self._kinect.close()
        pygame.quit()

# -- Kinect lecture -- Kinect lecture -- Kinect lecture -- Kinect lecture -- Kinect lecture 

def Kinect_lecture():
    global Kinect_Lecture
    game = BodyGameRuntime();
    game.run();
    Kinect_Lecture = True
    


# -- Global variables -- Global variables -- Global variables -- Global variables

Laser_obj = None # Object to manipulate the Hokuyo URG scan laser
Laser_lec = [] # List that contains the laser's samples
Laser_lecture = False # Variable for begin or finish the lecture of the LRF
Time_LRF = [] # List that contains the time of each sample of the LRF
Laser_scan_regitry = [] # List that each list scan

Kinect_Lecture = False # Variable for begin or finish the lecture of the Kinect
Kinect_data = [] # List that contains each sample of the LRF
Time_kinect = [] # List that contains the time of each sample of the Kinect


## -- Program's Main -- Program's Main -- Program's Main -- Program's Main -- Program's Main
   
if __name__ == "__main__":

    Data_name = input("File name: ")
    
    LRF_port = Find_port_URG_laser()
    start = 300
    stop = 468
    serial_comunication_speed = 256000

    if LRF_port != "Error":

        #global Laser

        # LRF object declaration
        
        Laser_obj = LRF(serial_comunication_speed,LRF_port,start,stop)
        Laser_obj.laser_on()

        # Thead LRF lecture begins
        
        Laser_lecture = True
        try:
            thread_LRF_lecture = threading.Thread(target=LRF_Lecture)
            thread_LRF_lecture.daemon = True
            thread_LRF_lecture.start()
        except:
            Laser_obj.laser_off()
            print("thread of the LRF lecture does not work")
        
        # Initialize the figure to plot
        
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='polar')
        ani= animation.FuncAnimation(fig, animate, interval=20)

        # Thead Kinect lecture begins
        try:
            thread_kinect_lecture = threading.Thread(target=Kinect_lecture)
            thread_kinect_lecture.start()
        except:
            Laser_obj.laser_off()
            print("thread of the Kinect lecture does not work")
        


        # Show the animate plot
        
        fig.show()

        Laser_lecture = False
        Laser_obj.laser_off()

        while not(Kinect_Lecture):
            pass
        

        # Save_data

        Excel_data = xlwt.Workbook(encoding="utf-8")

        # Save_LRF_Data

        sheet_LRF_data = Excel_data.add_sheet("Data_LRF")
        
        sheet_LRF_data.write(0,0,"Time (s)")
        for x in range(0,len(Laser_scan_regitry[0])):
            sheet_LRF_data.write(0,x+1,"Step " + str(start+x))

        for x in range(0,len(Laser_scan_regitry[0])):
            sheet_LRF_data.write(1,x+1,Laser_scan_regitry[0][x][0])

        if len(Time_LRF) == 0:
            initial_LRF_time = 0
        else:
            initial_LRF_time = Time_LRF[0]
        for n_scans  in  range(0,len(Laser_scan_regitry)):
            for n_samples_in_a_scan in range(0,len(Laser_scan_regitry[n_scans])+1):

                if n_samples_in_a_scan == 0:
                    sheet_LRF_data.write(n_scans+2, n_samples_in_a_scan, Time_LRF[n_scans] - initial_LRF_time)
                else:
                    sheet_LRF_data.write(n_scans+2, n_samples_in_a_scan, Laser_scan_regitry[n_scans][n_samples_in_a_scan-1][1])

        # Save_Kinect_Data


        sheet_kinect_data = Excel_data.add_sheet("Data_Kinect")
        sheet_kinect_data.write(0,0,"Time (s)")
        style = xlwt.easyxf('align: horz center')
        
        

        Kinect_names = ["SpineBase", "SpineMid", "Neck", "Head", "ShoulderLeft",
                        "ElbowLeft" , "WristLeft", "HandLeft" ,"ShoulderRight", "ElbowRight",
                        "WristRight","HandRight","HipLeft","KneeLeft ","AnkleLeft",
                        "FootLeft","HipRight","KneeRight","AnkleRight","FootRight",
                        "SpineShoulder","HandTipLeft","ThumbLeft","HandTipRight","ThumbRight"]

        p=1  
        for x in range (0,25):
            sheet_kinect_data.write_merge(0, 0, p, p+6, Kinect_names[x],style)
            for jump in range (0,7):
                if jump == 0:
                    sheet_kinect_data.write_merge(1,1,p,p+2,"Position",style)       
                    sheet_kinect_data.write(2,p,"X pos")
                elif jump == 1:
                    sheet_kinect_data.write(2,p,"Y pos")
                elif jump == 2:
                    sheet_kinect_data.write(2,p,"Z pos")
                elif jump == 3:
                    sheet_kinect_data.write_merge(1,1,p,p+3,"Orientation",style)
                    sheet_kinect_data.write(2,p,"Q1 pos")
                elif jump == 4:
                    sheet_kinect_data.write(2,p,"Q2 pos")
                elif jump == 5:
                   sheet_kinect_data.write(2,p,"Q3 pos")
                elif jump == 6:
                    sheet_kinect_data.write(2,p,"Q4 pos")
                p=p+1
                
        if len(Time_kinect) == 0:
            Initial_time_kinect = 0
        else:
            Initial_time_kinect = Time_kinect[0]
        for x in range (0,len(Kinect_data)):
            p=1
            for y in range(0,len(Kinect_data[x])+1):
                if y ==0:
                    sheet_kinect_data.write(x+3,y,Time_kinect[x] - Initial_time_kinect)
                else:
                    
                    for jump in range (0,7):       
                        sheet_kinect_data.write(x+3,p,Kinect_data[x][y-1][jump])
                        p=p+1
                    
                    
                        

        

        Excel_data.save( "LRF and Kinect data " + Data_name + ".xls")
        print("The data has been saved")


    else:
        print("The program doesn't work")
