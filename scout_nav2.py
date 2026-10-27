# -*- coding: utf-8 -*-
"""
Created on Wed Dec  3 17:34:57 2025

@author: frank
"""

import time
import math
from tcr_roboclaw import Roboclaw #library created for French robotics championship which uses the official RoboClaw library
import multiprocessing as mp

"""beginning of global variables"""

"""motor velocity PID constants"""
velP=0.82800
velI=0.04293
velD=0.00000
"""motor position PID constants"""
posP=33.60066
posI=0.00000
posD=0.00000

"""current settings"""
maxcurrent=1500 #max current is 15A per motor. Multiply the max current in Amps by 100. Max=1500
M1blanking=6554 #6554->20%, % of filtering of the current on channel M1 when in low PWM to prevent max_current error. Range from 0 6554 (0 to 20%)
M2blanking=6554 #6554->20%, % of filtering of the current on channel M2 when in low PWM to prevent max_current error. Range from 0 6554 (0 to 20%)

"""physical constraints/properties of the robot"""
wheel_radius=0.0775 #radius of the wheel in m
wheel_dist=0.48316 #distance between the 2 wheels in m
reduct_ratio=25 #reduction ratio of the motor
posmax=800*reduct_ratio #maximum quadrature pulses for 1 turn of the the wheel
anglemin=(2*math.pi)/posmax #maximum resolution of the motors in rad


"""max encoder speed"""
max_enc_speed=62000 #max qpps to be measured at the beginning of the setup using the find_enc_mean_speed(x) function

"""tolerance of error before accepting new order"""
theta_tol_er=0.001
d_tol_er=0.005
thresh_dist=1
thresh_theta=40*math.pi/180

"""end of global variables"""

def read_pos()->[int,int]:
    try:
        curr_pos1,curr_pos2=roboclaw.ReadEncM1()[1],roboclaw.ReadEncM2()[1]
    except:
        curr_pos1,curr_pos2=0,0
    return curr_pos1,curr_pos2

def read_speed()->[int,int]:
    try:
        curr_speed1,curr_speed2=roboclaw.ReadSpeedM1()[1],roboclaw.ReadSpeedM2()[1]
    except:
        curr_speed1,curr_speed2=0,0
    return curr_speed1,curr_speed2

def read_buffers()->[int,int]:
    curr_buff1,curr_buff2=roboclaw.ReadBufferLengths()[1],roboclaw.ReadBufferLengths()[2]
    return curr_buff1,curr_buff2

def rad_to_deg(X=0)->float:
    return (180/math.pi)*X

def deg_to_rad(X=0)->float:
    return (math.pi/180)*X

def ang_to_qp(X=0)->int:
    return deg_to_rad(X)/anglemin

def rad_to_qp(X=0)->int:
    return X/anglemin

def qp_to_ang(X=0)->float:
    return rad_to_deg(X*anglemin)

def qp_to_rad(X=0)->float:
    return X*anglemin

def dist_to_rad(X=0)->float:
    return X/wheel_radius

def rad_to_dist(X=0)->float:
    return X*wheel_radius

def qp_to_dist(X=0)->float:
    return X*wheel_dist*anglemin

def dist_to_qp(X=0)->float:
    return X/(wheel_radius*anglemin)

def rad_to_orient(X=0)->float:
    return X%(2*math.pi)

def orient_pos(X=0)->float:
    if(X<0):
        return X+(2*math.pi)
    else:
        return X

def integ(uk_past,ek,t,t_past,bounded:bool=False,upper=0,lower=0)->float:
    integ=uk_past+(t-t_past)*ek
    if(bounded==False):
        return integ
    else:
        if(integ>=upper):
            return uk_past+upper
        elif(integ<=lower):
            return uk_past+lower
        else:
            return integ

def deriv(ek,ek_past,t,t_past)->float:
    if((t-t_past)==0):
        return 0
    else:
        return (ek-ek_past)/(t-t_past)

def print_values(x,y,x_ref,y_ref,theta,d_ref,d,theta_ref,rw_vel,lw_vel,rw_vel_ref,lw_vel_ref,t,t_past,output_distance,output_theta):
    print("x-coordinates:",x.value)
    print("y-coordinates:",y.value)
    print("x_ref:",x_ref.value)
    print("y_ref:",y_ref.value)
    print("theta:",rad_to_deg(theta.value))
    print("theta_ref:",rad_to_deg(theta_ref.value))
    print("d_ref:",d_ref.value)
    print("d:",d.value)
    print("rw_vel:",rw_vel.value)
    print("lw_vel:",lw_vel.value)
    print("rw_vel_ref:",rw_vel_ref.value)
    print("lw_vel_ref:",lw_vel_ref.value)
    print("delta_t:",t.value-t_past.value)
    print("output_distance:",output_distance.value)
    print("output_theta:",output_theta.value)
    print()
    
def find_enc_mean_speed(x=100)->int:
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
    
def set_max_current()->bool:
    if(not(roboclaw.SetM1MaxCurrent(maxcurrent) and roboclaw.SetM2MaxCurrent(maxcurrent))):
        print("failed to set max current")
        return False
    return True
    
def set_blanking_percent()->bool:
    if(not(roboclaw.SetCurrentBlankingPercentage(M1blanking,M2blanking))):
        print("failed to set current blaking %")
        return False
    return True
    
def stop_robot(deccel_ramp,dist=None,buffer=1)->bool:
    if(dist!=None):
        if(not(roboclaw.BuffSpeedAccelDistanceM1M2(deccel_ramp, 0, dist, 0, dist, buffer))):
            print("failed to execute buffspeedaccelpos")
            return False
    else:
        if(not(roboclaw.SpeedAccelM1M2(deccel_ramp, 0, 0))):
            print("failed to execute speedaccel")
            return False
        while (read_speed()[0]!=0 and read_speed()[1]!=0):
            continue
    return True

def update_readings()->bool:
    """update of time measurements"""
    t_past.value=t.value
    t.value=time.perf_counter()
    """updating measure variables"""
    old_phi_r=phi_r.value
    old_phi_l=phi_l.value
    phi_r.value=read_pos()[0] #relative reading of the right encoder since last movement in qp
    phi_l.value=read_pos()[1] #relative reading of the left encoder since last movement in qp
    print("phi r and l:",phi_r.value,phi_l.value)
    rw_vel.value=read_speed()[0] #speed reading of the right encoder in qpps
    lw_vel.value=read_speed()[1] #speed reading of the left encoder in qpps
    # theta.value=theta_0.value+(wheel_radius/wheel_dist)*(qp_to_rad(phi_r.value-phi_l.value)) #new z-axis angle of the robot in world referential in rad, can also be measured instead of calculated if there is a sensor for it
    v.value=(wheel_radius/2)*qp_to_rad(rw_vel.value+lw_vel.value) #average of the linear velocities of both wheels of the robot in m/s
    vx.value=(math.cos(theta.value))*v.value #x-axis speed of the robot in the world referential in m/s
    vy.value=(math.sin(theta.value))*v.value #y-axis speed of the robot in the world referential in m/s
    d.value+=(wheel_radius/2)*qp_to_rad((phi_r.value-old_phi_r)+(phi_l.value-old_phi_l)) #measured distance the robot has travelled so far in m
    # x.value+=(wheel_radius/2)*qp_to_rad((phi_r.value-old_phi_r)+(phi_l.value-old_phi_l))*math.cos(theta.value) #x-axis position of the robot in the world referential in m
    # y.value+=(wheel_radius/2)*qp_to_rad((phi_r.value-old_phi_r)+(phi_l.value-old_phi_l))*math.sin(theta.value) #y-axis position of the robot in the world referential in m
    return True

def PID_d(lock,state,errord,output_distance,bound=True,upper=1.5,lower=-1.5,dt=0.01)->bool:
    import time
    """PID constants"""
    kPd=0.7
    kId=0
    kDd=0.05
    """time constants"""
    t=0.0
    t_past=0.0
    """error, integral and derivative values"""
    errord_past=0.0
    integd=0.0
    """infinite loop to be performed by the PID"""
    while(1):
        if(not(state.value)):
            break
        else:
            """update of time measurements"""
            t_past=t
            t=time.perf_counter()
            """PID output computation"""
            errord_past=errord.value
            integd=integ(integd, errord.value, t, t_past)
            derivd=deriv(errord.value, errord_past, t, t_past)
            output_distance.value=kPd*errord.value+kId*integd+kDd*derivd
            if(bound==True):
                if(output_distance.value>=upper):
                    output_distance.value=upper
                elif(output_distance.value<=lower):
                    output_distance.value=lower
        """ensures almost constant sampling period"""
        time.sleep(dt)
    return True

def PID_t(lock,state,errort,output_theta,bound=False,upper=(math.pi),lower=-(math.pi),dt=0.01)->bool:
    import time
    """PID constants"""
    kPt=1.0
    kIt=0
    kDt=0.1
    """time constants"""
    t=0.0
    t_past=0.0
    """error, integral and derivative values"""
    errort_past=0.0
    integt=0.0
    """infinite loop to be performed by the PID"""
    while(1):
        if(not(state.value)):
            break
        else:
            """update of time measurements"""
            t_past=t
            t=time.perf_counter()
            """PID output computation"""
            errort_past=errort.value
            integt=integ(integt, errort.value, t, t_past)
            derivt=deriv(errort.value, errort_past, t, t_past)
            output_theta.value=kPt*errort.value+kIt*integt+kDt*derivt
            if(bound==True):
                if(output_theta.value>=upper):
                    output_theta.value=upper
                elif(output_theta.value<=lower):
                    output_theta.value=lower
        """ensures almost constant sampling period"""
        time.sleep(dt)
    return True

def to_robot(accel,m1_vel,m2_vel)->bool:
    roboclaw.SpeedAccelM1M2(accel, int(m1_vel), int(m2_vel))
    return True

"""innit function"""
def innit()->bool:
    print(roboclaw.ReadVersion())
    roboclaw.ResetEncoders() #resets the value of both encoders to 0
    set_max_current() #sets the max current in each motors
    set_blanking_percent() #filters the current noise of the motors when in low PWM duty cycle
    roboclaw.SetM1VelocityPID(velP, velI, velD, max_enc_speed)
    roboclaw.SetM2VelocityPID(velP, velI, velD, max_enc_speed)
    """makes sure both motors are in e-stop"""
    roboclaw.DutyM1M2(0, 0)
    time.sleep(2) #makes sure the settings have time to be properly set
    return True

def target(accel:int=62000,dt:float=0.01)->bool:
    """initiate the different processes in their dedicated cores using the multiprocessing library"""
    update_readings()
    errort.value=theta_ref.value-theta.value
    if(abs(errort.value)>math.pi):
        if(errort.value<0):
            errort.value=errort.value+(2*math.pi)
        else:
            errort.value=errort.value-(2*math.pi)
    d.value=0
    errord.value=d_ref.value-d.value
    """create processes to be perfromed in parallel"""
    pid_d=mp.Process(target=PID_d,args=(lock,state,errord,output_distance))
    pid_t=mp.Process(target=PID_t,args=(lock,state,errort,output_theta))
    """starts the created mulriprocesses in parallel"""
    pid_d.start()
    pid_t.start()
    with lock:
        while(((abs(d_ref.value-d.value))>thresh_dist) and not(lin_vel.value==0)):
            update_readings()
            theta.value=orient_pos(rad_to_orient(theta.value))
            errort.value=theta_ref.value-theta.value
            if(abs(errort.value)>math.pi):
                if(errort.value<0):
                    errort.value=errort.value+(2*math.pi)
                else:
                    errort.value=errort.value-(2*math.pi)
            errord.value=d_ref.value-d.value
            biasd.value=(1-abs(errort.value/math.pi))**7
            rw_vel_ref.value=rad_to_qp((1/wheel_radius)*((biasd.value*lin_vel.value)+((wheel_dist/2)*output_theta.value))) #in qqps
            lw_vel_ref.value=rad_to_qp((1/wheel_radius)*((biasd.value*lin_vel.value)-((wheel_dist/2)*output_theta.value))) #in qpps)
            to_robot(accel, rw_vel_ref.value, lw_vel_ref.value)
            print_values(x, y, x_ref, y_ref, theta, d_ref, d, theta_ref, rw_vel, lw_vel, rw_vel_ref, lw_vel_ref, t, t_past, output_distance, output_theta)
        while((abs(theta_ref.value-theta.value)>theta_tol_er) or (abs(d_ref.value-d.value))>d_tol_er):
            update_readings()
            theta.value=orient_pos(rad_to_orient(theta.value))
            errort.value=theta_ref.value-theta.value
            if(abs(errort.value)>math.pi):
                if(errort.value<0):
                    errort.value=errort.value+(2*math.pi)
                else:
                    errort.value=errort.value-(2*math.pi)
            errord.value=d_ref.value-d.value
            biasd.value=(1-abs(errort.value/math.pi))**7
            rw_vel_ref.value=rad_to_qp((1/wheel_radius)*((biasd.value*output_distance.value)+((wheel_dist/2)*output_theta.value))) #in qqps
            lw_vel_ref.value=rad_to_qp((1/wheel_radius)*((biasd.value*output_distance.value)-((wheel_dist/2)*output_theta.value))) #in qpps
            to_robot(accel, rw_vel_ref.value, lw_vel_ref.value)
            print_values(x, y, x_ref, y_ref, theta, d_ref, d, theta_ref, rw_vel, lw_vel, rw_vel_ref, lw_vel_ref, t, t_past, output_distance, output_theta)
    with lock:
        state.value=0
    """all parallel processes are closed"""
    pid_d.join()
    pid_t.join()
    with lock:
        state.value=1
    return True

def arc(accel:int=62000,dt:float=0.01)->bool:
    """initiate the different processes in their dedicated cores using the multiprocessing library"""
    update_readings()
    print_values(x, y, x_ref, y_ref, theta, d_ref, d, theta_ref, rw_vel, lw_vel, rw_vel_ref, lw_vel_ref, t, t_past, output_distance, output_theta)
    d.value=0
    theta_ref.value=theta.value+theta_arc.value
    errort.value=theta_ref.value-theta.value
    """create processes to be perfromed in parallel"""
    pid_t=mp.Process(target=PID_t,args=(lock,state,errort,output_theta))
    """starts the created mulriprocesses in parallel"""
    pid_t.start()
    with lock:
        while((abs(theta_ref.value-theta.value)>thresh_theta) and not(wz.value==0)):
            update_readings()
            errort.value=theta_ref.value-theta.value
            rw_vel_ref.value=rad_to_qp((1/wheel_radius)*((wz.value*r_ref.value)+((wheel_dist/2)*wz.value))) #in qqps
            lw_vel_ref.value=rad_to_qp((1/wheel_radius)*((wz.value*r_ref.value)-((wheel_dist/2)*wz.value))) #in qqps
            to_robot(accel, rw_vel_ref.value, lw_vel_ref.value)
            print_values(x, y, x_ref, y_ref, theta, d_ref, d, theta_ref, rw_vel, lw_vel, rw_vel_ref, lw_vel_ref, t, t_past, output_distance, output_theta)
        while(abs(theta_ref.value-theta.value)>theta_tol_er):
            update_readings()
            errort.value=theta_ref.value-theta.value
            rw_vel_ref.value=rad_to_qp((1/wheel_radius)*((output_theta.value*r_ref.value)+((wheel_dist/2)*output_theta.value))) #in qqps
            lw_vel_ref.value=rad_to_qp((1/wheel_radius)*((output_theta.value*r_ref.value)-((wheel_dist/2)*output_theta.value))) #in qqps
            to_robot(accel, rw_vel_ref.value, lw_vel_ref.value)
            print_values(x, y, x_ref, y_ref, theta, d_ref, d, theta_ref, rw_vel, lw_vel, rw_vel_ref, lw_vel_ref, t, t_past, output_distance, output_theta)
        state.value=0
    """all parallel processes are closed"""
    pid_t.join()
    with lock:
        state.value=1
    return True

def set_ref(d_set=0,x_set=0,y_set=0,theta_set=0,r=0,w=0,vel=0)->bool:  
    with lock:    
        update_readings()
        x_ref.value=x_set
        y_ref.value=y_set
        if((x_set==0) and (y_set==0)):
            theta.value=rad_to_orient(theta.value)
            d_ref.value=d_set
            theta_ref.value=theta.value
        else:
            d_ref.value=math.sqrt(((x_ref.value-x.value)**2)+((y_ref.value-y.value)**2))
            theta_ref.value=orient_pos(rad_to_orient(math.atan2((y_ref.value-y.value), (x_ref.value-x.value))))
        if(vel>qp_to_dist(max_enc_speed)):
            lin_vel.value=1.5
        else:
            lin_vel.value=vel
        theta_arc.value=theta_set
        r_ref.value=r
        if(theta_arc.value>0):
            wz.value=w
        else:
            wz.value=-w
    return True

def calibration()->bool:
    roboclaw.DutyM1M2(-100, -100)
    print("cal started")
    time.sleep(1)
    update_readings()
    while((abs(rw_vel.value)>0) or (abs(lw_vel.value)>0)):
        update_readings()
    roboclaw.DutyM1M2(0, 0)
    time.sleep(1)
    with lock:
        x.value=0
        y.value=0.4
    roboclaw.ResetEncoders()
    time.sleep(1)
    return True


"""main() function"""
if(__name__=="__main__"):
    """Innitiation"""
    """initial setup to start serial connection to the RoboClaw using USB"""
    roboclaw=Roboclaw("COM9", 115200)
    roboclaw.open()
    if(not(innit())):
        print("Innit has failed, reload programm!")
    else:
        print("Innit sucessfull")
    state=mp.Value('i',1)
    """     variables used in multiprocessing"""
    theta_0=mp.Value('d',math.pi/2) #offset anlge of the robot in the world referential in rad when beginning code
    """time constants"""
    t=mp.Value('d',0.0)
    t_past=mp.Value('d',0.0)
    
    """present and past informations to be updated"""
    phi_r=mp.Value('d',0.0) #angle of right wheel in qp
    phi_l=mp.Value('d',0.0) #angle of left wheel in qp
    x=mp.Value('d',0.0) #x-axis position of the robot in world referential in m
    y=mp.Value('d',0.0) #y-axis position of the robot in world referential in m
    rw_vel=mp.Value('d',0.0) #right wheel angular velocity in qpps
    lw_vel=mp.Value('d',0.0) #left wheel angular velocity in qpps
    vx=mp.Value('d',0.0) #x-axis linear velocity of the robot in world referential in m/s
    vy=mp.Value('d',0.0) #y-axis linear velocity of the robot in world referential in m/s
    v=mp.Value('d',0.0) #average of the linear speeds of the wheels of the robot
    
    """reference values"""
    rw_vel_ref=mp.Value('d',0.0) #reference of the angle velocity of the left wheel in qpps
    lw_vel_ref=mp.Value('d',0.0) #reference of the angle velocity of the left wheel in qpps
    x_ref=mp.Value('d',0.0)
    y_ref=mp.Value('d',0.0)
    r_ref=mp.Value('d',0.0)
    theta_arc=mp.Value('d',0.0)
    lin_vel=mp.Value('d',0.0)
    wz=mp.Value('d',0.0) #z-axis angular velocity of the robot in world referential in m/s

    """PID d"""
    d_ref=mp.Value('d',0.0) #reference value of the distance to be travelled by the robot in m
    d=mp.Value('d',0.0) #measured distance travelled by the robot in m
    errord=mp.Value('d',0.0)
    output_distance=mp.Value('d',0.0) #output of the distance PID. It is the average of the linear velocities of both wheels of the robot in m/s
    biasd=mp.Value('d',1) #bias gain value to be applied to the distance PID which also needs to be updated every measures
    
    """PID theta"""
    theta_ref=mp.Value('d',0.0) #reference of angle of the trajectory in the world referential in rad
    theta=mp.Value('d',0.0) #z-axis angle of the robot in world referential in rad
    errort=mp.Value('d',0.0)
    output_theta=mp.Value('d',0.0) #output of the orientation (theta) PID. It is the angular velocity of the robot in the z-axis of the world referential in rad/s
    """     end of multiprocessing variables"""
    
    """lock needed to protected the shared variables"""
    lock=mp.RLock()
    
    square=[set_ref,arc,set_ref,target,set_ref,arc,set_ref,target,set_ref,arc,set_ref,target,set_ref,arc,set_ref,target,]
    
    """movement functions for the robot"""
    # calibration()
    # set_ref(x_set=0,y_set=0.6)
    # target()
    # set_ref(r=0,theta_set=deg_to_rad(-90))
    # arc()
    # for i in range(1):
    #     square[0](r=0.4,theta_set=deg_to_rad(90),w=0.1)
    #     square[1]()
    #     square[2](x_set=0.4,y_set=2.51,vel=0.2)
    #     square[3]()
    #     square[4](r=0.4,theta_set=deg_to_rad(90),w=0.1)
    #     square[5]()
    #     square[6](x_set=-5.84,y_set=2.91,vel=0.2)
    #     square[7]()
    #     square[8](r=0.4,theta_set=deg_to_rad(90),w=0.1)
    #     square[9]()
    #     square[10](x_set=-6.24,y_set=0.95,vel=0.2)
    #     square[11]()
    #     square[12](r=0.4,theta_set=deg_to_rad(90),w=0.1)
    #     square[13]()
    #     square[14](x_set=0,y_set=0.55,vel=0.2)
    #     square[15]()
    # set_ref(r=0,theta_set=deg_to_rad(90),w=0.2)
    # arc()
    # set_ref(x_set=-1,y_set=0.55,vel=0.3)
    # target()
    # set_ref(r=0,theta_set=deg_to_rad(-180),w=0.2)
    # arc()
    # set_ref(x_set=0,y_set=0.55,vel=0.3)
    # target()
    # set_ref(d_set=5,vel=1)
    # target()
    # set_ref(r=0,theta_set=deg_to_rad(180))
    # arc()
    # set_ref(d_set=5,vel=1)
    # target()
    roboclaw.DutyM1M2(0, 0)
print("main finished!")