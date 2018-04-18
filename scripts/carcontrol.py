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
        self.car_pub = rospy.Publisher("car_input", car_input, queue_size = 10)

        self.speed = 0
        self.path = []
        self.control = []
        x_start = 0
        y_start = 0

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
        self.vicon_sub = rospy.Subscriber("/vrpn_client_node/rc_car/pose", PoseStamped, self.callback, queue_size = 10)

        mode = input("Select Vehicle Motion? 1) Move Straight 2) Straight Path (PID) 3) Curve Path (PID)")
        speed_text = input("Select Speed (s/f)")
        if speed_text == 'f':
            self.speed = 1.0
        else:
            self.speed = 0.5
        if mode == '1':  # Drive straight with no controller
            angle = 0
            self.move(angle)
        elif mode == '2':  # Drive straight with PID Control
            angle = 0

            # Setup Path
            straight_path_file = 'path_SRC\straight_path.csv'
            straight_line = load_file(straight_path_file)
            self.path = Path(straight_line, x_start, y_start)

            # Setup PID
            kp = 0.1
            ki = 0.001
            kd = 2.8
            self.control = PID(kp, ki, kd)

            self.move(angle)
        elif mode == '3':  # Drive allow curved path with PID Control
            angle = 20

            # Setup Path
            curve_path_file = 'path_SRC\circle_path.csv'
            curved_line = load_file(curve_path_file)
            self.path = Path(curved_line, x_start, y_start)

            # Setup PID
            kp = 0.1
            ki = 0.001
            kd = 2.8
            self.control = PID(kp, ki, kd)

            self.move(angle)
        else:
            print('Nothing selected')

    def callback(self,data):
        self.pos = data.pose.position
        self.coordinates = np.append(self.coordinates, np.matrix([self.pos.x, self.pos.y, self.pos.z]), axis=0)
        np.savetxt("src/py_test/data.csv", self.coordinates, delimiter=",")

        # Find error
        error = self.path.find_error([self.pos.x, self.pos.y])
        self.control.update_error(error)

        # Update steering angle
        angle = self.control.output_steering()
        self.move(angle)

    def move(self, angle):
        Z.steer_angle = angle
        Z.power = self.speed
        self.car_pub.publish(Z)


def main(args):
    rospy.init_node('CarControl', anonymous=True, disable_signals=True)
    print("here")
    cc = CarControl()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")


if __name__ == '__main__':
    main(sys.argv)