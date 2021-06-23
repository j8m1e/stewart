from vpython import *
from vpython.no_notebook import stop_server
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
import struct
import serial as ser
import serial.tools.list_ports
ports = serial.tools.list_ports.comports()
print(ports)
arduino = ser.Serial('/dev/ttyUSB0', baudrate = 115200, timeout=.1) #assuming serial connection is (COM3)



for port, desc, hwid in sorted(ports):
        print("{}: {} [{}]".format(port, desc, hwid))

def transform (rot,trans):
    r = R.from_quat(rot)
    if np.allclose(rot,np.array([0,0,0,1]),rtol=1e-08):
        return P_init + trans
    else:
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
    h = 30
    d = 120
    l = P_pivot - B_servo
    e = 2*h*l[:,2]
    # print(e)
    f = 2*h*(np.cos(B_angle)*l[:,0]+np.sin(B_angle)*l[:,1])
    # print(f)
    g = np.sum(l**2,axis = 1)-(d**2-h**2)
    # print(g)
    # print(g/(np.sqrt(e**2+f**2)))
    # print(np.arcsin(g/(np.sqrt(e**2+f**2))))
    a = (np.arcsin(g/(np.sqrt(e**2+f**2)))-np.arctan2(f,e))*np.array([1,-1,1,-1,1,-1])
    # print(a)
    return a
    # return np.fmod(ang_inc + servoangle,2*np.pi)

def rotate_arm():

    r = R.from_quat(get_quat(servoangle,servo_axis))
    return r.apply(B_init - B_servo)+B_servo

Hoffset = -120.
# P_vertex = [(-60.000,44.880,0),(-8.868,74.402,0),(68.868, 29.521,0),(68.868,-29.521,0),(-8.868,-74.402,0),(-60.000,-44.880,0)]
P_init =  np.array([[-45.094,58.105,0],[-27.774,68.105,0],[72.868, 10,0],[72.868,-10,0],[-27.774,-68.105,0],[-45.094,-58.105,0]])
B_servo = np.array([[-74.575,49.167,Hoffset],[-5.293,89.167,Hoffset],[79.868, 40,Hoffset],[79.868,-40,Hoffset],[-5.293,-89.167,Hoffset],[-74.575,-49.167,Hoffset]])
B_init = np.array([[-100.556,34.167,Hoffset],[20.668,104.167,Hoffset],[79.868,70,Hoffset],[79.868,-70,Hoffset],[20.668,-104.167,Hoffset],[-100.556,-34.167,Hoffset]])
P_pivot = np.array([None] * 6)
rot = np.array([0,0,0,1])
trans = np.array([0,0,0])
vect_init = np.array([0,0,1])
Pangle = math.radians(10)
axis = np.array([1,0,0])
B_angle = np.radians(np.array([120,120,0,0,240,240]))
servo_axis = np.array([(math.cos(math.radians(120)),math.sin(math.radians(120)),0),(math.cos(math.radians(120)),math.sin(math.radians(120)),0),(1,0,0),(1,0,0),(math.cos(math.radians(240)),math.sin(math.radians(240)),0),(math.cos(math.radians(240)),math.sin(math.radians(240)),0)])
inc = math.radians(2)
ang_inc = np.array([math.radians(2)*(-1)**i for i in range(6)])
servoangle = np.array([0 for i in range(6)])
t = 0
circle_ang = 0
P_pivot = transform(rot,trans)
print(np.linalg.norm(B_init-P_init,axis = 1))

platform = curve(pos = P_pivot.tolist(),color = color.yellow)
platform.append(tuple(P_pivot[0]))
servo = curve(pos = B_servo.tolist())
servo.append(tuple(B_servo[0]))
c = [None]*6
s = [None]*6
arm = [[P_pivot[i].tolist(),B_init[i].tolist()]for i in range(0,6)]
servo = [[B_init[i].tolist(),B_servo[i].tolist()]for i in range(0,6)]
for i in range(0,6):
    c[i] = curve(pos = arm[i], color = color.cyan)
    s[i] = curve(pos = servo[i], color = color.red)


while t<12000:
    rate(30)
    circle_ang = (circle_ang+inc)%(2*math.pi)
    trans = np.array([10*math.sin(1*circle_ang),10*math.cos(2*circle_ang),10*math.cos(3*circle_ang)])
    axis = np.array([math.cos(circle_ang),math.sin(circle_ang),0])
    rot = get_quat(Pangle,axis)
    P_pivot = transform(rot,trans)
    platform.visible = False
    platform = curve(pos = P_pivot.tolist(),color = color.yellow)
    platform.append(tuple(P_pivot[0]))
    servoangle = get_arm_ang(P_pivot,B_init,B_servo,B_angle)
    arduinoangle = np.asarray(np.degrees(servoangle).astype(int)+90)
    # print(arduinoangle)
    arduino.write(struct.pack('>BBBBBB',arduinoangle[0],arduinoangle[1],arduinoangle[2],arduinoangle[3],arduinoangle[4],arduinoangle[5]))
    # print (arduino.readline() )
    B_pivot = rotate_arm()
    # print(np.linalg.norm(B_pivot-P_pivot,axis = 1))
    arm = [[P_pivot[i].tolist(),B_pivot[i].tolist()]for i in range(0,6)]
    servo = [[B_pivot[i].tolist(),B_servo[i].tolist()]for i in range(0,6)]
    for i in range(0,6):
        c[i].visible = False
        c[i] = curve(pos = arm[i], color = color.cyan)
        # s[i].rotate(angle = ang_inc[i], axis = vector(servo_axis[i][0],servo_axis[i][1],servo_axis[i][2]), origin = vector(B_servo[i][0],B_servo[i][1],B_servo[i][2]))
        s[i].visible = False
        s[i] = curve(pos = servo[i], color = color.red)
    t = t+1


stop_server()