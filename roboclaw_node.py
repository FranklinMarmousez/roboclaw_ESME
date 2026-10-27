import math
import time
import rclpy
from rclpy.node import Node
from tcr_roboclaw import Roboclaw #library created for French robotics championship which uses the official RoboClaw library

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


"""beginning of global variables"""

"""motor velocity PID constants"""
velP=0.82800
velI=0.04293
velD=0.00000

"""current settings"""
maxcurrent=1500 #max current is 15A per motor. Multiply the max current in Amps by 100. Max=1500
M1blanking=6554 #6554->20%, % of filtering of the current on channel M1 when in low PWM to prevent max_current error. Range from 0 6554 (0 to 20%)
M2blanking=6554 #6554->20%, % of filtering of the current on channel M2 when in low PWM to prevent max_current error. Range from 0 6554 (0 to 20%)

"""physical constraints/properties of the robot"""
wheel_radius=0.0775 #radius of the wheel in m
wheel_dist=0.483 #distance between the 2 wheels in m
reduct_ratio=25 #reduction ratio of the motor
posmax=800*reduct_ratio #maximum quadrature pulses for 1 turn of the the wheel
anglemin=(2*math.pi)/posmax #maximum resolution of the motors in rad

"""max encoder speed"""
max_enc_speed=62000 #max qpps to be measured 


"""end of global variables"""
    

def set_max_current():
    """sets the maximum current which the roboclaw driver can take, by default, 15 Amps"""
    if(not(roboclaw.SetM1MaxCurrent(maxcurrent) and roboclaw.SetM2MaxCurrent(maxcurrent))):
        print("failed to set max current")
    

def set_blanking_percent():
    """creates a filter which prevents errors in low duty cycles for the PWM on the motors"""
    if(not(roboclaw.SetCurrentBlankingPercentage(M1blanking,M2blanking))):
        print("failed to set current blaking %")


def innit():
    print(roboclaw.ReadVersion())
    roboclaw.ResetEncoders() #resets the value of both encoders to be 0
    set_max_current()
    set_blanking_percent()
    roboclaw.SetM1VelocityPID(velP, velI, velD, max_enc_speed)
    roboclaw.SetM2VelocityPID(velP, velI, velD, max_enc_speed)
    roboclaw.DutyM1M2(0, 0) #makes sure both motors have no PWM
    time.sleep(2) #makes sure the settings have time to be properly set


class RoboclawOdomPublisher(Node):

    def _init_(self):
        super()._init_('roboclaw_odom_publisher')

        # paramètres robot
        self.wheel_radius=wheel_radius #in m
        self.wheel_separation=wheel_dist #in m
        self.ticks_per_rev=20000 #in qp (quadrature pulses, also known as ticks)
        self.meters_per_tick=(2*math.pi*self.wheel_radius)/self.ticks_per_rev

        # position robot
        self.x=0.0
        self.y=0.0
        self.theta=0.0
        self.theta_offset=0

        #encoder values
        self.last_right_ticks=0
        self.last_left_ticks=0

        # publisher odometry
        self.odom_pub=self.create_publisher('/odom',10)

        # broadcaster TF
        self.tf_broadcaster=TransformBroadcaster(self)

        # timer update
        self.timer=self.create_timer(0.01, self.update)
        self.last_time=self.get_clock().now()
        
    def read_encoders(self):
        right_ticks=roboclaw.ReadEncM1()[1] #current qp value of right wheel encoder
        left_ticks=roboclaw.ReadEncM2()[1] #current qp value of left wheel encoder
        return left_ticks, right_ticks
    
    
    def update(self):
        #update time values
        now=self.get_clock().now()
        dt=(now-self.last_time).nanoseconds*1e-9
        self.last_time=now

        #update encoder related values
        right_ticks,left_ticks=self.read_encoders()
        d_right=right_ticks-self.last_right_ticks
        d_left=left_ticks-self.last_left_ticks
        self.last_left_ticks=left_ticks
        self.last_right_ticks=right_ticks
        dvr=d_right*self.meters_per_tick
        dvl=d_left*self.meters_per_tick
        v=(dvr+dvl)/2
        dtheta=(dvr-dvl)/self.wheel_separation

        #update coordinates
        self.theta=self.theta_offset+(self.wheel_separation/self.wheel_radius)*(right_ticks-left_ticks)
        self.x+=self.v*math.cos(self.theta)
        self.y+=self.v*math.sin(self.theta)

        vx=v/dt
        wz=dtheta/dt

        self.publish_odom(now, vx, wz)

    
    def publish_odom(self, now, vx, wz):
        qz=math.sin(self.theta/2.0)
        qw=math.cos(self.theta/2.0)

        odom = Odometry()

        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz

        self.odom_pub.publish(odom)

        # TF odom -> base_link
        t = TransformStamped()

        t.header.stamp = now.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y

        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = RoboclawOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if (__name__ == '__main__'):
    roboclaw=Roboclaw("/dev/ttyACM0", 115200)
    roboclaw.open()
    innit()
    main()