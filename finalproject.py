from pyCreate2 import create2
import pyCreate2
import math
import odometry
import pid_controller
import lab8_map
import rrt_map
import particle_filter
import rrt
import numpy as np

def convert_point_to_pixels(point):
    x = point[0]
    y = 3.0 - point[1]
    x *= 100
    y *= 100
    return (int(x),int(y))

def convert_pixel_to_point(pixels):
    x = pixels[0]
    y = pixels[1]
    x /= 100
    y /= 100
    y = 3.0 - y
    return (x,y)

class Run:
    def __init__(self, factory):
        """Constructor.
        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.servo = factory.create_servo()
        self.sonar = factory.create_sonar()
        self.arm = factory.create_kuka_lbr4p()
        self.virtual_create = factory.create_virtual_create()
        # self.virtual_create = factory.create_virtual_create("192.168.1.XXX")
        self.odometry = odometry.Odometry()
        self.mapJ = lab8_map.Map("lab8_map.json")
        self.map = rrt_map.Map("configuration_space.png")
        self.rrt = rrt.RRT(self.map)

        # TODO identify good PID controller gains
        self.pidTheta = pid_controller.PIDController(90, 1, 60, [-3, 3], [-50, 50], step_size=10, is_angle=True)
        # TODO identify good particle filter parameters

        self.base_speed = 50

        self.joint_angles = np.zeros(7)

        # *******************************************************************************
        # SET THESE VARIABLES IF YOU CHANGE THE SIM
        # these are the start coordinates for the robot
        self.start_x = 1.0049
        self.start_y = 0.495
        # these are the coordinates for the arm
        self.arm_x = 1.6001
        self.arm_y = 3.3999
        # these are the goal coordinates for the robot
        self.goal_x = 1.5
        self.goal_y = 2.85
        # set the shelf number to 0, 1, 2 these are the shelfs
        # in range of the arm
        self.shelf_number = 1
        # *******************************************************************************
        # our assumptions about distances and positioning
        self.cup_height = 0.2
        self.gripper_len = 0.4
        # distance from ground to first joint
        self.arm_base_height = 0.3105
        # estimated using V-REP (joint2 - joint4)
        self.link1 = 0.4
        # estimated using V-REP (joint4 - joint6)
        self.link2 = 0.39
        # eyeballed these numbers
        self.shelf_heights = [0.21, 0.53, 0.85]

        self.pf = particle_filter.ParticleFilter(self.mapJ, 100, 0.06, 0.05, 0.05, self.start_x, self.start_y)


    def sleep(self, time_in_sec):
        """Sleeps for the specified amount of time while keeping odometry up-to-date
        Args:
            time_in_sec (float): time to sleep in seconds
        """
        start = self.time.time()
        while True:
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                # print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
            t = self.time.time()
            if start + time_in_sec <= t:
                break

    def go_to_angle(self, goal_theta):
        old_x = self.odometry.x
        old_y = self.odometry.y
        old_theta = self.odometry.theta
        start_time = self.time.time()
        prev_theta = old_theta
        goal_theta = math.atan2(math.sin(goal_theta), math.cos(goal_theta))      
        while True:
            theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))      
            output_theta = self.pidTheta.update(theta, goal_theta, self.time.time())
            self.create.drive_direct(int(+output_theta), int(-output_theta))
            angle_error = math.atan2(math.sin(goal_theta - theta), math.cos(goal_theta - theta))
            print(output_theta, theta, goal_theta, angle_error)
            
            if self.time.time() - start_time > 0.3 and abs(angle_error) < 0.005 and abs(theta - prev_theta) < 0.005:
                break
            if abs(goal_theta) > abs(theta) and angle_error < 0:
                break
            prev_theta = theta
            self.sleep(0.01)
        self.create.drive_direct(0, 0)
        self.sleep(2)
        self.pf.move_by(self.odometry.x - old_x, self.odometry.y - old_y, self.odometry.theta - old_theta)

    def forward(self):
        old_x = self.odometry.x
        old_y = self.odometry.y
        old_theta = self.odometry.theta
        base_speed = 100
        distance = 0.5
        goal_x = self.odometry.x + math.cos(self.odometry.theta) * distance
        goal_y = self.odometry.y + math.sin(self.odometry.theta) * distance
        while True:
            goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
            self.create.drive_direct(int(base_speed+output_theta), int(base_speed-output_theta))

            # stop if close enough to goal
            distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
            if distance < 0.05:
                self.create.drive_direct(0, 0)
                break
            self.sleep(0.01)
        self.pf.move_by(self.odometry.x - old_x, self.odometry.y - old_y, self.odometry.theta - old_theta)

    def visualize(self):
        x, y, theta = self.pf.get_estimate()
        self.virtual_create.set_pose((x, y, 0.1), theta)
        data = []
        for particle in self.pf._particles:
            data.extend([particle.x, particle.y, 0.1, particle.theta])
        self.virtual_create.set_point_cloud(data)

    def run(self):
        self.create.start()
        self.create.safe()
        self.arm.open_gripper()

        self.create.drive_direct(0, 0)

        # self.arm.open_gripper()
      
        posC = self.create.sim_get_position()
        print("inti pos")
        print(posC)
        # self.get_cup()

        #self.arm.go_to(1,np.pi/4)

        start_x = self.start_x
        start_y = self.start_y
        starting_position = convert_point_to_pixels((start_x, start_y))

        goal_position = convert_point_to_pixels((self.goal_x, self.goal_y))

        K = 5000
        delta = 10
        self.rrt.build(starting_position, K, delta)
        path = self.rrt.shortest_path(goal=self.rrt.nearest_neighbor(goal_position))
        for i in range(1, len(path)):
            self.map.draw_line(pos1=path[i-1].state, pos2=path[i].state, color=(255, 0, 0))
        # self.map.save("hello.png")
        print("map generated")

        #self.arm.close_gripper()

        print("setting up sensor reading and particle filter")
        # request sensors
        self.create.start_stream([
            pyCreate2.Sensor.LeftEncoderCounts,
            pyCreate2.Sensor.RightEncoderCounts,
        ])
        self.visualize()

        """
        print("Initial localizing routine!")
        for i in range(4):
            self.go_to_angle(self.odometry.theta + math.pi / 2)
            self.visualize()
            distance = self.sonar.get_distance()
            print("measured distance is: ", distance)
            self.pf.measure(distance, 0)
            self.visualize()
        """
        fifth_size = int(len(path)/8)
        
        print("----Starting path following----")
        self.odometry.x = start_x
        self.odometry.y = start_y
        left_speed = 0
        right_speed = 0
        prev_x = start_x
        prev_y = start_y
        prev_theta = self.odometry.theta
        for i in range(len(path)):
            goal_x, goal_y = convert_pixel_to_point(path[i].state)  # returns x,y
            print("wait point: ", goal_x,goal_y)
            while True:
                state = self.create.update()
                curr_time = self.time.time()
                if state is not None:
                    # update stuff
                    self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                    goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                    theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))
                    # print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))

                    distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                    if distance < 0.15:
                        break             
                    output_theta = self.pidTheta.update(theta, goal_theta, curr_time)

                    # base version:
                    left_speed = int(self.base_speed-output_theta)
                    right_speed = int(self.base_speed+output_theta)
                    self.create.drive_direct(right_speed, left_speed)

                    # improved version 2: fuse with velocity controller
                    # output_distance = self.pidDistance.update(0, distance, curr_time)
                    # self.create.drive_direct(int(output_theta + output_distance),int(-output_theta + output_distance))
                            
                    
            if i % fifth_size == 0:
                for j in range(4,0,-1):
                    self.create.drive_direct(right_speed*j/5, left_speed*j/5)
                    self.time.sleep(0.01)
                self.create.drive_direct(0,0)
                self.time.sleep(0.5)
                self.pf.move_by(self.odometry.x - prev_x, self.odometry.y - prev_y, self.odometry.theta - prev_theta)
                prev_x = self.odometry.x
                prev_y = self.odometry.y
                prev_theta = self.odometry.theta
                print("[x:{},y:{},theta:{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
                self.visualize()
                print("Updating particle filter!")
                distance = self.sonar.get_distance()
                print("measured distance is: ", distance)
                self.pf.measure(distance, 0)
                self.visualize()
                self.time.sleep(0.1)
            # print("new waypoint!")


        print("Should be at goal")
        self.create.drive_direct(0, 0)
        self.time.sleep(4)

        self.virtual_create.enable_buttons()
        self.visualize()
        # self.go_to_angle(math.radians(90))
        # self.sleep(5)
        self.get_cup()
        self.place_cup()

        # self.arm.go_to(4, math.radians(-90))
        self.time.sleep(4)

        while True:
            b = self.virtual_create.get_last_button()
            if b == self.virtual_create.Button.MoveForward:
                self.forward()
                self.visualize()
            elif b == self.virtual_create.Button.TurnLeft:
                self.go_to_angle(self.odometry.theta + math.pi / 2)
                self.visualize()
            elif b == self.virtual_create.Button.TurnRight:
                self.go_to_angle(self.odometry.theta - math.pi / 2)
                self.visualize()
            elif b == self.virtual_create.Button.Sense:
                distance = self.sonar.get_distance()
                print(distance)
                self.pf.measure(distance, 0)
                self.visualize()

            # posC = self.create.sim_get_position()
            # print(posC)
            # self.arm.go_to(4, math.radians(-90))
            # self.arm.go_to(5, math.radians(90))
            self.time.sleep(100)

            self.time.sleep(0.01)

    def forward_kinematics(self, theta1, theta2):
        self.arm.go_to(1, theta1)
        self.arm.go_to(3, theta2)
        L1 = 0.4 # estimated using V-REP (joint2 - joint4)
        L2 = 0.39 # estimated using V-REP (joint4 - joint6)
        z = L1 * math.cos(theta1) + L2 * math.cos(theta1 + theta2) + 0.3105
        x = L1 * math.sin(theta1) + L2 * math.sin(theta1 + theta2)
        print("Go to {},{} deg, FK: [{},{},{}]".format(math.degrees(theta1), math.degrees(theta2), -x, 0, z))


    def inverse_kinematics(self, x_i, z_i):

        L1 = self.link1
        L2 = self.link2
        # Corrections for our coordinate system
        z = z_i - self.arm_base_height
        x = -x_i
        # compute inverse kinematics
        r = math.sqrt(x*x + z*z)
        alpha = math.acos((L1*L1 + L2*L2 - r*r) / (2*L1*L2))
        theta2 = math.pi - alpha

        beta = math.acos((r*r + L1*L1 - L2*L2) / (2*L1*r))
        theta1 = math.atan2(x, z) - beta

        print(theta1,theta2)
        if theta2 < -9*math.pi/16 or theta2 > 9*math.pi/16 or theta1 < -9*math.pi/16 or theta1 > 9*math.pi/16:
            theta2 = math.pi + alpha
            theta1 = math.atan2(x, z) + beta
            print(theta1,theta2)
        if theta2 < -9*math.pi/16 or theta2 > 9*math.pi/16 or theta1 < -9*math.pi/16 or theta1 > 9*math.pi/16:
            print("Not possible")
            return

        self.arm.go_to(1, theta1)
        self.arm.go_to(3, theta2)
        self.arm.go_to(5, -(theta2-(math.radians(90)-theta1)))
        print("Go to [{},{}], IK: [{} deg, {} deg]".format(x_i, z_i, math.degrees(theta1), math.degrees(theta2)))

    def get_cup(self):
        print("cup [x:{},y:{},theta:{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))

        # using trig to calculate the coordinates of the cup
        c_x = self.odometry.x - 0.1372 * math.cos(self.odometry.theta)
        c_y = self.odometry.y - 0.1372 * math.sin(self.odometry.theta)

        angle = math.tan((c_x - self.arm_x)/(c_y - self.arm_y))
        self.arm.go_to(0, -1 * angle)
        h = math.sqrt((c_x - self.arm_x)**2+(c_y - self.arm_y)**2)
        print(h)
        self.time.sleep(6)
        self.inverse_kinematics(-h + self.gripper_len, self.cup_height)
        self.time.sleep(6)
        input("press enter to continue")
        #self.inverse_kinematics(-h+.32,.17)
        self.time.sleep(6)
        self.arm.close_gripper()
        self.time.sleep(6)
        input("press enter to continue")

    def place_cup(self):
        height = self.shelf_heights[self.shelf_number]
        self.inverse_kinematics(-0.5, height)
        self.sleep(4)
        self.arm.go_to(0, 1 * -math.pi/4)
        self.time.sleep(1)
        self.arm.go_to(0, 2 * -math.pi/4)
        self.time.sleep(1)
        self.arm.go_to(0, 3 * -math.pi/4)
