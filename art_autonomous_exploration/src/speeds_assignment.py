#!/usr/bin/env python

import rospy
import math
import time

from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from sonar_data_aggregator import SonarDataAggregator
from laser_data_aggregator import LaserDataAggregator
from navigation import Navigation

# Class for assigning the robot speeds 
class RobotController:

    # Constructor
    def __init__(self):
        
      # Debugging purposes
      self.print_velocities = rospy.get_param('print_velocities')

      # Where and when should you use this?
      self.stop_robot = False

      # Create the needed objects
      self.sonar_aggregation = SonarDataAggregator()
      self.laser_aggregation = LaserDataAggregator()
      self.navigation  = Navigation()

      self.linear_velocity  = 0
      self.angular_velocity = 0

      # Check if the robot moves with target or just wanders
      self.move_with_target = rospy.get_param("calculate_target")

      # The timer produces events for sending the speeds every 110 ms
      rospy.Timer(rospy.Duration(0.11), self.publishSpeeds)
      self.velocity_publisher = rospy.Publisher(\
              rospy.get_param('speeds_pub_topic'), Twist,\
              queue_size = 10)

    # This function publishes the speeds and moves the robot
    def publishSpeeds(self, event):
        
      # Produce speeds
      self.produceSpeeds()

      # Create the commands message
      twist = Twist()
      twist.linear.x = self.linear_velocity
      twist.linear.y = 0
      twist.linear.z = 0
      twist.angular.x = 0 
      twist.angular.y = 0
      twist.angular.z = self.angular_velocity

      # Send the command
      self.velocity_publisher.publish(twist)

      # Print the speeds for debuggind purposes
      if self.print_velocities == True:
      	print "[L,R] = [" + str(twist.linear.x) + " , " + \
            str(twist.angular.z) + "]"

    # Produces speeds from the laser
    def produceSpeedsLaser(self):
      scan = self.laser_aggregation.laser_scan
      linear  = 0
      angular = 0
 
      
      ############################### NOTE QUESTION ############################
      # Check what laser_scan contains and create linear and angular speeds
      # for obstacle avoidance
			
      # CHALLENGE 1 part1
      laz0r = self.laser_aggregation

      linearCor = 0 
      angularCor = 0 
      for i in range(1,len(scan)):
        angularCor -= math.sin(laz0r.angle_min + i* laz0r.angle_increment) / scan[i]**2
        if i >= 0.4 * len(scan) and i<= 0.6*len(scan):
          linearCor -= math.cos(laz0r.angle_min + i* laz0r.angle_increment) / scan[i]**2
# For testing purposes         
      tstPub = rospy.Publisher('test0', Twist, queue_size=10)
      tstTwst = Twist()
      tstTwst.linear.x = linearCor
      tstTwst.linear.y = linearCor/60 * 0.3
      tstTwst.linear.z = 0
      tstTwst.angular.x = 0
      tstTwst.angular.y = angularCor/100 * 0.3
      tstTwst.angular.z = angularCor
      tstPub.publish(tstTwst)
# For testing purposes         

      linearCor = (linearCor / 60) * 0.3
      angularCor = (angularCor / 100) * 0.3
      
      linearCor = max(linearCor, -0.3)

      if linearCor >= -0.1:
        angularCor = math.copysign(linearCor, angularCor) 
      else: 
        if angularCor >= 0.3 :
          angularCor = 0.3
        elif angularCor <= -0.3:
          angularCor = -0.3
        elif abs(angularCor)<=0.04:
          angularCor *= 3 

      
			# ##########################################################################
      return [linearCor, angularCor]

    # Combines the speeds into one output using a motor schema approach
    def produceSpeeds(self):
 
      # Produce target if not existent
      if self.move_with_target == True and \
              self.navigation.target_exists == False:

        # Create the commands message
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        # Send the command
        self.velocity_publisher.publish(twist)
        self.navigation.selectTarget()

      # Get the submodule's speeds
      [l_laser, a_laser] = self.produceSpeedsLaser()
      
      # You must fill these
      self.linear_velocity  = 0
      self.angular_velocity = 0
      
      if self.move_with_target == True:
        [l_goal, a_goal] = self.navigation.velocitiesToNextSubtarget()
        ############################### NOTE QUESTION ############################
        # You must combine the two sets of speeds. You can use motor schema,
        # subsumption of whatever suits your better.

        # CHALLENGE 4

        # if abs(l_laser) <= 0.05 and abs(a_laser) < 0.25:
        #   self.linear_velocity = l_goal
        #   self.angular_velocity = a_goal
        # else:
        #   self.linear_velocity = 0.65 * l_laser + 0.35 * l_goal
        #   self.angular_velocity = 0.65 * a_laser + 0.35 * a_goal 
 
        if abs(l_laser) <= 0.15:
          self.linear_velocity = l_goal
          self.angular_velocity = a_goal
        else:
          self.linear_velocity = l_goal + 0.5* (l_goal / 0.3) * l_laser 
          self.angular_velocity = a_goal + 0.5* (a_goal / 0.3) * a_laser  

				##########################################################################
      else:
        ############################### NOTE QUESTION ############################
        # Implement obstacle avoidance here using the laser speeds.
        # Hint: Subtract them from something constant

        # CHALLENGE 1 part2

				self.linear_velocity = 0.3 + l_laser
				self.angular_velocity = a_laser        
				pass
        ##########################################################################

    # Assistive functions
    def stopRobot(self):
      self.stop_robot = True

    def resumeRobot(self):
      self.stop_robot = False
