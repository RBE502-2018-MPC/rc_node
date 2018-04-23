#!/usr/bin/env python

import sys, time
import rospy
import numpy as np
from load_file import load_file
from PID import PID
from Path import Path
from rc_node.msg import car_input
from geometry_msgs.msg import PoseStamped, TwistStamped


class CarControl:

    def __init__(self):
        # Publish the car input data
        self.car_pub = rospy.Publisher("car_input", car_input, queue_size=10)
        self.R = rospy.Rate(1)

        self.speed = 0
        self.path = []
        self.control = []
        self.pos = []
        self.coordinates = np.empty((0, 2))
        self.error = np.empty((0, 1))
        self.steering_angle = np.empty((0, 1))
        self.Z = car_input()
        x_start = 0
        y_start = 0
        self.start_time = 0

        self.tracking = False
        self.driving = False

        # For moving the rc car at fixed velocity
        # elapsed_time = 0.0
        # start_time = time.time()
        # while elapsed_time < 1.5:
        # 	elapsed_time = time.time() - start_time
        # 	Z = car_input()
        # 	Z.steer_angle = 0
        # 	Z.power = 0.5
        # 	print "publishing"
        # 	self.car_pub.publish(Z)

        # Subscribing to position data from vicon
        self.vicon_sub = rospy.Subscriber("/vrpn_client_node/rc_car/pose", PoseStamped, self.callback, queue_size=10)

        # mode = raw_input("Select Vehicle Motion? 1) Move Straight 2) Straight Path (PID) 3) Curve Path (PID)")
        # speed_text = raw_input("Select Speed (s/f)")
        mode = '3'
        speed_text = 's'
        if speed_text == 'f':
            self.speed = 0.6
            kp = 12.0  # 0.1 12
            ki = 0.01  # 0.001 0.01
            kd = 7.0  # 2.8 7.0
        else:
            self.speed = 0.35  # < 0.3 is reverse so use 3.5
            kp = 12.0  # 0.1 12
            ki = 0.001  # 0.001 0.01
            kd = 7.0  # 2.8 7.0

        if mode == '1':  # Drive straight with no controller
            print('Driving in circle no PID')
            angle = -18.1
            self.move(angle)
        elif mode == '2':  # Drive straight with PID Control
            angle = 0

            # Setup Path
            straight_path_file = 'rc_node/scripts/path_SRC/straight_path_2m.csv'
            straight_line = load_file(straight_path_file)
            self.path = Path(straight_line, x_start, y_start)

            # Setup PID
            self.control = PID(kp, ki, kd)
            self.tracking = True

            self.move(angle)
        elif mode == '3':  # Drive allow curved path with PID Control
            angle = -18.1

            # Setup Path
            curve_path_file = 'rc_node/scripts/path_SRC/circle_path.csv'
            curved_line = load_file(curve_path_file)
            self.path = Path(curved_line, x_start, y_start)

            # Setup PID
            self.control = PID(kp, ki, kd)
            self.tracking = True

            self.move(angle)
        elif mode == '4':  # Drive allow figure 8 path with PID Control
            angle = -18.1  # this angle will result in a 0.25 m radius circle

            # Setup Path
            figure8_path_file = 'rc_node/scripts/path_SRC/figure8_path_16x.csv'
            curved_line = load_file(figure8_path_file)
            self.path = Path(curved_line, x_start, y_start)

            # Setup PID
            self.control = PID(kp, ki, kd)
            self.tracking = True

            self.move(angle)
        else:
            print('Nothing selected')

    def callback(self, data):
        self.pos = data.pose.position

        if not self.driving:  # Adjust path to current position, need to add rotation.
                x_start = self.pos.x
                y_start = self.pos.y
                if self.tracking:
                    self.path.reset(x_start, y_start)
                self.driving = True
                self.start_time = time.time()

        if self.tracking:
            # Find error
            error = self.path.find_error([self.pos.x, self.pos.y])
            print(error)
            self.control.update_error(error)

            # Update steering angle
            angle = self.control.output_steering()
        else:
            angle = self.Z.steer_angle

        if (time.time() - self.start_time) > 10.0:
            self.stop()
            angle = 0
        # np.savetxt("rc_node/test.csv", self.coordinates, delimiter=",")
        self.move(angle)

        # Up arrays to save
        print(self.error.shape)
        self.coordinates = np.append(self.coordinates, np.matrix([self.pos.x, self.pos.y]), axis=0)
        self.error = np.append(self.error, np.matrix([error]), axis=0)
        self.steering_angle = np.append(self.steering_angle, np.matrix([angle]), axis=0)

    def move(self, angle):
        if angle > 46:
            angle = 46
        elif angle < -46:
            angle = -46

        self.Z.steer_angle = angle
        self.Z.power = self.speed
        self.car_pub.publish(self.Z)
        # self.R.sleep()  # prevents from publishing to fast
        print(self.Z)

    def stop(self):
        print('stopping')
        self.speed = 0
        self.Z.steer_angle = 0
        self.Z.power = 0
        self.car_pub.publish(self.Z)
        self.driving = False


def main(args):
    rospy.init_node('CarControl', anonymous=True, disable_signals=True)
    print("Staring")
    cc = CarControl()
    try:
        print('try')
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        #cc.stop()
       # rospy.spin()
    finally:
        # clean up
        print('done')


if __name__ == '__main__':
    main(sys.argv)
