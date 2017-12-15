#!/usr/bin/env python

__author__ = 'thaus'

import rospy
from pid import PID
from geometry_msgs.msg import Vector3, Vector3Stamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32, Float64
from morus_msgs.msg import PIDController
from dynamic_reconfigure.server import Server
from morus_msgs.cfg import MavAttitudeCtlParamsConfig
import math, copy
from datetime import datetime
from rosgraph_msgs.msg import Clock
import numpy

class AttitudeControl:
    '''
    Class implements MAV attitude control (roll, pitch, yaw). Two PIDs in cascade are
    used for each degree of freedom.
    Subscribes to:
        /morus/imu                - used to extract attitude and attitude rate of the vehicle
        /morus/mot_vel_ref        - used to receive referent motor velocity from the height controller
        /morus/euler_ref          - used to set the attitude referent (useful for testing controllers)
    Publishes:
        /morus/command/motors     - referent motor velocities sent to each motor controller
        /morus/pid_roll           - publishes PID-roll data - referent value, measured value, P, I, D and total component (useful for tuning params)
        /morus/pid_roll_rate      - publishes PID-roll_rate data - referent value, measured value, P, I, D and total component (useful for tuning params)
        /morus/pid_pitch          - publishes PID-pitch data - referent value, measured value, P, I, D and total component (useful for tuning params)
        /morus/pid_pitch_rate     - publishes PID-pitch_rate data - referent value, measured value, P, I, D and total component (useful for tuning params)
        /morus/pid_yaw            - publishes PID-yaw data - referent value, measured value, P, I, D and total component (useful for tuning params)
        /morus/pid_yaw_rate       - publishes PID-yaw_rate data - referent value, measured value, P, I, D and total component (useful for tuning params)

    Dynamic reconfigure is used to set controllers param online.
    '''

    def __init__(self):
        '''
        Initialization of the class.
        '''

        self.start_flag = False             # flag indicates if the first measurement is received
        self.config_start = False           # flag indicates if the config callback is called for the first time
        self.euler_mv = Vector3()           # measured euler angles
        self.euler_sp = Vector3(0, 0, 0)    # euler angles referent values

        self.w_sp = 0                       # referent value for motor velocity - it should be the output of height controller

        self.euler_rate_mv = Vector3()      # measured angular velocities

        self.clock = Clock()

        self.pid_roll = PID()                           # roll controller
        self.pid_roll_rate  = PID()                     # roll rate (wx) controller

        self.pid_pitch = PID()                          # pitch controller
        self.pid_pitch_rate = PID()                     # pitch rate (wy) controller

        self.pid_yaw = PID()                            # yaw controller
        self.pid_yaw_rate = PID()                       # yaw rate (wz) controller

        ##################################################################
        ##################################################################
        # Add your PID params here

        self.pid_roll.set_kp(0.0004) # 0.001
        self.pid_roll.set_ki(0.0)
        self.pid_roll.set_kd(0)
        self.pid_roll.set_dead_zone(0.0)

        self.pid_roll_rate.set_kp(0.0)
        self.pid_roll_rate.set_ki(0.0)
        self.pid_roll_rate.set_kd(0)
        self.pid_roll_rate.set_lim_high(1.0)
        self.pid_roll_rate.set_lim_low(-1.0)

        self.pid_pitch.set_kp(0.0004)
        self.pid_pitch.set_ki(0)
        self.pid_pitch.set_kd(0.0)
        self.pid_pitch.set_dead_zone(0.0)

        self.pid_pitch_rate.set_kp(0.0)
        self.pid_pitch_rate.set_ki(0.0)
        self.pid_pitch_rate.set_kd(0)
        self.pid_pitch_rate.set_lim_high(1.0)
        self.pid_pitch_rate.set_lim_low(-1.0)

        self.pid_yaw.set_kp(0)
        self.pid_yaw.set_ki(0)
        self.pid_yaw.set_kd(0)

        self.pid_yaw_rate.set_kp(0)
        self.pid_yaw_rate.set_ki(0)
        self.pid_yaw_rate.set_kd(0)

        self.joint0 = [0, -45, -60,
                       0, -45, -60,
                       0, -45, -60,
                       0, -45, -60]

        self.joint_ref = self.joint0
        self.joint_msg = JointState();

        self.joint_pos = [0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0]

        self.ml = 1.494
        self.mb = 2.564
        self.M = self.mb + 4 * self.ml
        self.Sx = 0.02
        self.Sz = 0.02
        self.D = 0.424
        self.nu = 1/2.564

        self.Jacobian = numpy.matrix([[0.0,0.0,0.0,0.0], [0.0,0.0,0.0,0.0]])
        self.inv_Jacobian = numpy.matrix([[0.0,0.0],[0.0,0.0], [0.0,0.0],[0.0,0.0]])
        self.u_meas = numpy.matrix([[0.0],[0.0],[0.0],[0.0]])
        self.dp_ref = numpy.matrix([[0.0], [0.0]])
        self.du_ref = numpy.matrix([[0.0], [0.0], [0.0], [0.0]])
        print self.u_meas


        ##################################################################
        ##################################################################

        self.rate = 20.0
        self.ros_rate = rospy.Rate(self.rate)                 # attitude control at 20 Hz

        self.t_old = 0

        rospy.Subscriber('imu', Imu, self.ahrs_cb)
        rospy.Subscriber('euler_ref', Vector3, self.euler_ref_cb)
        rospy.Subscriber('/clock', Clock, self.clock_cb)
        rospy.Subscriber('aquashoko_joint_positions', JointState, self.joint_cb)

        self.pub_joint_references = rospy.Publisher('aquashoko_chatter', JointState, queue_size=1)
        self.pub_pid_roll = rospy.Publisher('pid_roll', PIDController, queue_size=1)
        self.pub_pid_roll_rate = rospy.Publisher('pid_roll_rate', PIDController, queue_size=1)
        self.pub_pid_pitch = rospy.Publisher('pid_pitch', PIDController, queue_size=1)
        self.pub_pid_pitch_rate = rospy.Publisher('pid_pitch_rate', PIDController, queue_size=1)
        #self.pub_pid_yaw = rospy.Publisher('pid_yaw', PIDController, queue_size=1)
        #self.pub_pid_yaw_rate = rospy.Publisher('pid_yaw_rate', PIDController, queue_size=1)
        self.cfg_server = Server(MavAttitudeCtlParamsConfig, self.cfg_callback)

    def run(self):
        '''
        Runs ROS node - computes PID algorithms for cascade attitude control.
        '''

        while rospy.get_time() == 0:
            print 'Waiting for clock server to start'

        print 'Received first clock message'

        while not self.start_flag:
            print "Waiting for the first measurement."
            rospy.sleep(0.5)
        print "Starting attitude control."

        self.t_old = rospy.Time.now()
        clock_old = self.clock
        #self.t_old = datetime.now()
        self.count = 0
        self.loop_count = 0

        while not rospy.is_shutdown():
            self.ros_rate.sleep()
            
            clock_now = self.clock
            dt_clk = (clock_now.clock - clock_old.clock).to_sec()

            clock_old = clock_now
            if dt_clk > (1.0 / self.rate + 0.01):
                self.count += 1
                print self.count, ' - ',  dt_clk

            if dt_clk < (1.0 / self.rate - 0.01):
                self.count += 1
                print self.count, ' - ',  dt_clk

            self.u_meas[0,0] = 0 #math.radians((self.joint_pos[0] - self.joint_pos[6]) / 2.0)
            self.u_meas[1,0] = 0 #math.radians((self.joint_pos[3] - self.joint_pos[9]) / 2.0)
            self.u_meas[2,0] = 0 #math.radians((self.joint_pos[1] - self.joint_pos[7]) / 2.0)
            self.u_meas[3,0] = 0 #math.radians((self.joint_pos[4] - self.joint_pos[10]) / 2.0)
            #self.compute_jacobian()
            #self.inv_Jacobian = numpy.linalg.pinv(self.Jacobian)
            self.inv_Jacobian = numpy.matrix([[1.0*12.0,0.25*25.0],[0.25*-25.0, 1.0*12.0], [-25.0,0.0],[0.0,-25.0]])

            delta_y_ref = self.pid_roll.compute(self.euler_sp.x, self.euler_mv.x, dt_clk)
            # roll rate pid compute
            #roll_cmd = self.pid_roll_rate.compute(roll_rate_sv, self.euler_rate_mv.x, dt_clk)

            delta_x_ref = -self.pid_pitch.compute(self.euler_sp.y, self.euler_mv.y, dt_clk)
            # pitch rate pid compute
            #pitch_cmd = self.pid_pitch_rate.compute(pitch_rate_sv, self.euler_rate_mv.y, dt_clk)
            #print self.u_meas
            #print self.inv_Jacobian

            self.dp_ref[0,0] = delta_x_ref
            self.dp_ref[1,0] = delta_y_ref

            self.du_ref = self.inv_Jacobian * self.dp_ref

            #du3_ref = self.inv_Jacobian[2,0] * delta_x_ref + self.inv_Jacobian[2,1] * delta_y_ref
            #du4_ref = self.inv_Jacobian[3,1] * delta_y_ref + self.inv_Jacobian[3,0] * delta_x_ref

              # Publish joint references

            #if self.loop_count < 5:
            self.joint_ref[1] = math.degrees(self.du_ref[2,0]) + self.joint0[1]
            self.joint_ref[7] = math.degrees(-self.du_ref[2,0]) + self.joint0[7]
            self.joint_ref[4] = math.degrees(self.du_ref[3,0]) + self.joint0[4]
            self.joint_ref[10] = math.degrees(-self.du_ref[3,0]) + self.joint0[10]

            self.joint_ref[0] = math.degrees(self.du_ref[0,0]) + self.joint0[0]
            self.joint_ref[6] = math.degrees(-self.du_ref[0,0]) + self.joint0[6]
            self.joint_ref[3] = math.degrees(self.du_ref[1,0]) + self.joint0[3]
            self.joint_ref[9] = math.degrees(-self.du_ref[1,0]) + self.joint0[9]

            self.joint_msg.header.stamp = rospy.Time.now()
            self.joint_msg.position = copy.deepcopy(self.joint_ref)
            self.pub_joint_references.publish(self.joint_msg)
         
            # Publish PID data - could be usefule for tuning
            self.pub_pid_roll.publish(self.pid_roll.create_msg())
            self.pub_pid_pitch.publish(self.pid_pitch.create_msg())

            self.loop_count = self.loop_count + 1


    def compute_jacobian(self):
        c1 = math.cos(self.u_meas[0,0])
        s1 = math.sin(self.u_meas[0,0])
        c2 = math.cos(self.u_meas[1,0])
        s2 = math.sin(self.u_meas[1,0])
        c3 = math.cos(self.u_meas[2,0])
        s3 = math.sin(self.u_meas[2,0])
        c4 = math.cos(self.u_meas[3,0])
        s4 = math.sin(self.u_meas[3,0])

        self.Jacobian[0,0] = self.compute_j00_j11(s1, s3, c1)
        self.Jacobian[1,1] = self.compute_j00_j11(s2, s4, c2)
        self.Jacobian[0,1] = -1.0 * self.compute_j01_j10_j02_j13(c2, c4)
        self.Jacobian[1,0] = self.compute_j01_j10_j02_j13(c1, c3)
        self.Jacobian[0,2] = -1.0 * self.compute_j01_j10_j02_j13(c1, c3)
        self.Jacobian[1,3] = -1.0 * self.compute_j01_j10_j02_j13(c2, c4)
        self.Jacobian[0,3] = self.compute_j03_j12(s2, s4)
        self.Jacobian[1,2] = -1.0 * self.compute_j03_j12(s1, s3)

    def compute_j00_j11(self, s1,s2, c1):
        res = self.ml * (math.sqrt(2) * s1 * s2 * (8 * self.ml * self.Sx + self.mb * (self.D*(1-self.nu) + 2 * self.Sx)) / self.M + 4 * c1 * self.Sz) / \
                (2 * (self.nu * self.mb + 4 * self.ml))
        return res

    def compute_j01_j10_j02_j13(self, c1,c2):
        res = (c1 * c2 * self.ml * (8 * self.ml * self.Sx + self.mb * (self.D*(1-self.nu) + 2 * self.Sx))) / \
                (math.sqrt(2) * self.M * (self.nu * self.mb + 4 * self.ml))
        return res

    def compute_j03_j12(self, s1,s2):
        res = (s1 * s2 * self.ml * (8 * self.ml * self.Sx + self.mb * (self.D*(1-self.nu) + 2 * self.Sx))) / \
                (math.sqrt(2) * self.M * (self.nu * self.mb + 4 * self.ml))
        return res
 
    def ahrs_cb(self, msg):
        '''
        AHRS callback. Used to extract roll, pitch, yaw and their rates.
        We used the following order of rotation - 1)yaw, 2) pitch, 3) roll
        :param msg: Type sensor_msgs/Imu
        '''
        if not self.start_flag:
            self.start_flag = True

        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        # conversion quaternion to euler (yaw - pitch - roll)
        self.euler_mv.x = math.atan2(2 * (qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz)
        self.euler_mv.y = math.asin(2 * (qw * qy - qx * qz))
        self.euler_mv.z = math.atan2(2 * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz)

        # gyro measurements (p,q,r)
        p = msg.angular_velocity.x
        q = msg.angular_velocity.y
        r = msg.angular_velocity.z

        sx = math.sin(self.euler_mv.x)   # sin(roll)
        cx = math.cos(self.euler_mv.x)   # cos(roll)
        cy = math.cos(self.euler_mv.y)   # cos(pitch)
        ty = math.tan(self.euler_mv.y)   # cos(pitch)

        # conversion gyro measurements to roll_rate, pitch_rate, yaw_rate
        self.euler_rate_mv.x = p + sx * ty * q + cx * ty * r
        self.euler_rate_mv.y = cx * q - sx * r
        self.euler_rate_mv.z = sx / cy * q + cx / cy * r

    def euler_ref_cb(self, msg):
        '''
        Euler ref values callback.
        :param msg: Type Vector3 (x-roll, y-pitch, z-yaw)
        '''
        self.euler_sp = msg

    def clock_cb(self, msg):
        self.clock = msg

    def joint_cb(self, msg):
        self.joint_pos = copy.deepcopy(msg.position)
        #print self.joint_pos

    def cfg_callback(self, config, level):
        """ Callback for dynamically reconfigurable parameters (P,I,D gains for each controller)
        """

        if not self.config_start:
            # callback is called for the first time. Use this to set the new params to the config server
            config.roll_kp = self.pid_roll.get_kp()
            config.roll_ki = self.pid_roll.get_ki()
            config.roll_kd = self.pid_roll.get_kd()

            config.roll_r_kp = self.pid_roll_rate.get_kp()
            config.roll_r_ki = self.pid_roll_rate.get_ki()
            config.roll_r_kd = self.pid_roll_rate.get_kd()

            config.pitch_kp = self.pid_pitch.get_kp()
            config.pitch_ki = self.pid_pitch.get_ki()
            config.pitch_kd = self.pid_pitch.get_kd()

            config.pitch_r_kp = self.pid_pitch_rate.get_kp()
            config.pitch_r_ki = self.pid_pitch_rate.get_ki()
            config.pitch_r_kd = self.pid_pitch_rate.get_kd()

            config.yaw_kp = self.pid_yaw.get_kp()
            config.yaw_ki = self.pid_yaw.get_ki()
            config.yaw_kd = self.pid_yaw.get_kd()

            config.yaw_r_kp = self.pid_yaw_rate.get_kp()
            config.yaw_r_ki = self.pid_yaw_rate.get_ki()
            config.yaw_r_kd = self.pid_yaw_rate.get_kd()

            self.config_start = True
        else:
            # The following code just sets up the P,I,D gains for all controllers
            self.pid_roll.set_kp(config.roll_kp)
            self.pid_roll.set_ki(config.roll_ki)
            self.pid_roll.set_kd(config.roll_kd)

            self.pid_roll_rate.set_kp(config.roll_r_kp)
            self.pid_roll_rate.set_ki(config.roll_r_ki)
            self.pid_roll_rate.set_kd(config.roll_r_kd)

            self.pid_pitch.set_kp(config.pitch_kp)
            self.pid_pitch.set_ki(config.pitch_ki)
            self.pid_pitch.set_kd(config.pitch_kd)

            self.pid_pitch_rate.set_kp(config.pitch_r_kp)
            self.pid_pitch_rate.set_ki(config.pitch_r_ki)
            self.pid_pitch_rate.set_kd(config.pitch_r_kd)

            self.pid_yaw.set_kp(config.yaw_kp)
            self.pid_yaw.set_kp(config.yaw_kp)
            self.pid_yaw.set_ki(config.yaw_ki)
            self.pid_yaw.set_kd(config.yaw_kd)

            self.pid_yaw_rate.set_kp(config.yaw_r_kp)
            self.pid_yaw_rate.set_ki(config.yaw_r_ki)
            self.pid_yaw_rate.set_kd(config.yaw_r_kd)

        # this callback should return config data back to server
        return config

if __name__ == '__main__':

    rospy.init_node('aquashoko_control')
    attitude_ctl = AttitudeControl()
    attitude_ctl.run()
