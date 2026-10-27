# -*- coding: utf-8 -*-
"""
Created on Wed Dec  3 17:34:57 2025

@author: frank
"""

import time
import math
from pynput.keyboard import Key, Listener #library to have a keylistenner to have a dynamic user interaction to control the driver
from tcr_roboclaw import Roboclaw #library created for French robotics championship which uses the official RoboClaw library

"""beginning of global variables"""

"""velocity PID constants"""
velP=0.82800
velI=0.04293
velD=0.00000
"""position PID constants"""
posP=33.60066
posI=0.00000
posD=0.00000

"""current settings"""
maxcurrent=1500 #max current is 15A per motor. Multiply the max current in Amps by 100. Max=1500
M1blanking=2621 #2621->8%, % of filtering of the current on channel M1 when in low PWM to prevent max_current error. Range from 0 6554 (0 to 20%)
M2blanking=6554 #6554->20%, % of filtering of the current on channel M2 when in low PWM to prevent max_current error. Range from 0 6554 (0 to 20%)

"""physical constraints/properties of the robot"""
wheel_radius=7.5
enc_precision=200
anglemin=2*math.pi/enc_precision
posmin=0
posmax=800
reductratio=25

"""max encoder speed"""
max_enc_speed=55000 #to be calculated at the beginning of the setup using the find_enc_mean_speed(x) function

"""reference velocities"""
vx=0 #linear velocity on x-axis of the world referential
vy=0 #linear velocity on y-axis of the world referential
wz=0 #angular velocity on z-axis of the world referential

M1=0
M2=0

"""end of global variables"""

"""initial setup to start serial connection to the RoboClaw using USB"""
roboclaw = Roboclaw("COM9", 9600)
roboclaw.open()

"""set of functions which can be used to control the robot"""
def current_pos():
    curr_pos1=roboclaw.ReadEncM1()[1]
    curr_pos2=roboclaw.ReadEncM2()[1]
    return curr_pos1,curr_pos2

def current_speed():
    curr_speed1=roboclaw.ReadSpeedM1()[1]
    curr_speed2=roboclaw.ReadSpeedM2()[1]
    return curr_speed1,curr_speed2

def calc_pos(dist):
    turns=0
    pos=dist/(anglemin*wheel_radius)
    if pos>posmax:
        turns=pos%posmax
        pos-=posmax*turns
    return turns,pos

def print_enc(x):
    for i in range(x):
        print("encoders value: ",roboclaw.ReadEncM1())
        print("encoders speed: ",roboclaw.ReadSpeedM1())
        print("encoders value: ",roboclaw.ReadEncM2())
        print("encoders speed: ",roboclaw.ReadSpeedM2())
    
def find_enc_mean_speed(x):
    meanM1=0
    meanM2=0
    roboclaw.ForwardM1(127)
    roboclaw.ForwardM2(127)
    time.sleep(1)
    for i in range(x):
        meanM1+=roboclaw.ReadSpeedM1()[1]
        meanM2+=roboclaw.ReadSpeedM2()[1]
    roboclaw.ForwardM1(127)
    roboclaw.ForwardM2(127)
    maxM1=meanM1//x
    maxM2=meanM2//x
    max_enc_speed=(maxM1+maxM2)//2
    time.sleep(2)
    print("the max encoder speed in QPPS is: ",max_enc_speed)
    roboclaw.ResetEncoders()
    return max_enc_speed
    
    
def set_vel_PID(kp,kd,ki,qpps):
    roboclaw.SetM1VelocityPID(kp, ki, kp, qpps)
    
def simple_demo():
    print("M1 and M2 encoder values: ",current_pos())
    print("buffer length is: ",roboclaw.ReadBufferLengths()) 
    roboclaw.BuffSpeedAccelDeccelPositionM1M2(10000, 55000, 10000, 60000, 10000, 55000, 10000, 60000,0)
    while (current_pos()[0]!=60000 and current_pos()[1]!=60000):
        continue
    print("buffer length is: ",roboclaw.ReadBufferLengths())
    print("M1 and M2 encoder values: ",current_pos())
    roboclaw.BuffSpeedAccelDeccelPositionM1M2(10000, 20000, 10000, 0, 10000, 20000, 10000, 0,0)
    while (current_pos()[0]!=0 and current_pos()[1]!=0):
         continue
    time.sleep(1)
    print("buffer length is: ",roboclaw.ReadBufferLengths())
    print("M1 and M2 encoder values: ",current_pos())
    roboclaw.BuffSpeedAccelDeccelPositionM1M2(10000, -10000, 10000, -60000, 10000, 10000, 10000, 60000,0)
    roboclaw.BuffSpeedAccelDeccelPositionM1M2(10000, 10000, 10000, 0, 10000, -10000, 10000, 0,0)
    
def simple_demo_keyboard(key):
    global M1,M2
    if key==key.up:
        M1+=10
        M2+=10
        decision(M1,M2)
    if key==key.down:
        M1+=-10
        M2+=-10
        decision(M1,M2)
    if key==key.left:
        M1+=10
        M2+=-10
        decision(M1,M2)
    if key==key.right:
        M1+=-10
        M2+=10
        decision(M1,M2)
    
def decision(M1,M2):
    if M1>0 and M2>0:
            roboclaw.ForwardM1(abs(M1))
            roboclaw.ForwardM2(abs(M2))
    elif M1<0 and M2>0:
        roboclaw.BackwardM1(abs(M1))
        roboclaw.ForwardM2(abs(M2))
    elif M1>0 and M2<0:
        roboclaw.BackwardM2(abs(M2))
        roboclaw.ForwardM1(abs(M1))
    elif M1<0 and M2<0:
        roboclaw.BackwardM2(abs(M2))
        roboclaw.BackwardM1(abs(M1))
    elif M1==0:
        roboclaw.ForwardM1(0)
    elif M2==0:
        roboclaw.SpeedM2(0)
    print("M1: ",M1,"M2: ",M2)
    
def set_max_current():
    roboclaw.SetM1MaxCurrent(maxcurrent)
    roboclaw.SetM2MaxCurrent(maxcurrent)
    
def set_blanking_percent():
    roboclaw.SetCurrentBlankingPercentage(M1blanking,M2blanking)
    
def stop_func():
    roboclaw.SpeedAccelM1M2(3000, 0, 0)
    while (current_speed()[0]!=0 and current_speed()[1]!=0):
        continue
    
def innit():
    print(roboclaw.ReadVersion())
    roboclaw.ResetEncoders() #resets the value of both encoders to 0
    set_max_current() #sets the max current in each motors
    set_blanking_percent() #filters the current noise of the motors when in low PWM duty cycle
    roboclaw.SetM1PositionPID(38, 0, 0, 0, 0, 0, 60000)
    roboclaw.SetM1VelocityPID(velP, velI, velD, max_enc_speed)
    roboclaw.SetM2PositionPID(38, 0, 0, 0, 0, 0, 60000)
    roboclaw.SetM2VelocityPID(velP, velI, velD, max_enc_speed)
    """makes sure both motors are not moving anymore"""
    roboclaw.ForwardM1(0)
    roboclaw.ForwardM2(0)
    time.sleep(2) #makes sure the settings have time to be properly set
    
"""Innitiation of main program"""
innit()

"""main program"""
# with Listener(on_press = simple_demo_keyboard) as listener:
#     listener.join()
simple_demo()
roboclaw.SpeedAccelM1M2(3000, 0, 0) #stops the motors gently if is not stopped yet