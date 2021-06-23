
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
import struct
import serial as ser
arduino = ser.Serial('/dev/ttyUSB0', baudrate = 115200, timeout=.1) #assuming serial connection is (COM3)



def transform (rot,trans):
    r = R.from_quat(rot)
    return r.apply(P_init) + trans




def get_quat(angle, axis) :
    rot = np.sin(angle/2).reshape(-1,1)*axis
    temp = np.cos(angle/2)
    if rot.size == 3:
        rot = np.append(rot,temp)
    else:
        temp = temp.reshape(-1,1)
        rot = np.hstack((rot,temp))
    return rot


def get_arm_ang(P_pivot,B_init,B_servo,B_angle):

    l = P_pivot - B_servo
    e = 2*h*l[:,2]
    f = 2*h*(np.cos(B_angle)*l[:,0]+np.sin(B_angle)*l[:,1])
    g = np.sum(l**2,axis = 1)-(d**2-h**2)
    a = (np.arcsin(g/(np.sqrt(e**2+f**2)))-np.arctan2(f,e))*np.array([1,-1,1,-1,1,-1])
    return a


#################DEFAULT VALUE######################
P_pivot = np.array([None] * 6)
rot = np.array([0,0,0,1])
trans = np.array([0,0,0])
vect_init = np.array([0,0,1])
Pangle = math.radians(10)
axis = np.array([1,0,0])
inc = math.radians(0.5)
ang_inc = np.array([math.radians(2)*(-1)**i for i in range(6)])
servoangle = np.array([0 for i in range(6)])
circle_ang = 0



#################PLATFORM SPECIFIC##################
Hoffset = -120.
h = 30
d = 120
P_init =  np.array([[-45.094,58.105,0],[-27.774,68.105,0],[72.868, 10,0],[72.868,-10,0],[-27.774,-68.105,0],[-45.094,-58.105,0]])
B_servo = np.array([[-74.575,49.167,Hoffset],[-5.293,89.167,Hoffset],[79.868, 40,Hoffset],[79.868,-40,Hoffset],[-5.293,-89.167,Hoffset],[-74.575,-49.167,Hoffset]])
B_init = np.array([[-100.556,34.167,Hoffset],[20.668,104.167,Hoffset],[79.868,70,Hoffset],[79.868,-70,Hoffset],[20.668,-104.167,Hoffset],[-100.556,-34.167,Hoffset]])
B_angle = np.radians(np.array([120,120,0,0,240,240]))




while True:

    #generate default movement
    circle_ang = (circle_ang+inc)%(2*math.pi)
    trans = np.array([10*math.sin(1*circle_ang),10*math.cos(2*circle_ang),10*math.cos(3*circle_ang)])
    axis = np.array([math.cos(circle_ang),math.sin(circle_ang),0])

    #calculation
    rot = get_quat(Pangle,axis)
    P_pivot = transform(rot,trans)
    servoangle = get_arm_ang(P_pivot,B_init,B_servo,B_angle)
    arduinoangle = np.asarray(np.degrees(servoangle).astype(int)+90)
    arduino.write(struct.pack('>BBBBBB',arduinoangle[0],arduinoangle[1],arduinoangle[2],arduinoangle[3],arduinoangle[4],arduinoangle[5]))
    arduino.readline()
