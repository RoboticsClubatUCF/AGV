"""The purpose of this program is to simulate real-world sensor output by adding
   adding noise to the sensor gazebo simulator sensor outputs."""

import random as rand
import rospy
import math
from nav_msgs.msg import Odometry

class Noise:
	def __init__(self):
		#Normal distribution inits. Initialize as std. normal distribution
		self.mean = 0 
		self.deviation = 1

	def app_normal(self, quiet_del_x, quiet_del_y, dev):
		#Set the deviation
		self.deviation = dev
		#Apply some noise onto the del values 
		noisy_del_x = quiet_del_x + rand.gauss(self.mean, self.deviation)
		noisy_del_y = quiet_del_y + rand.gauss(self.mean, self.deviation)

		return noisy_del_x, noisy_del_y

class Topic_callbacks:
	def __init__(self):
		#Odometry inits
		self.prev_x = 0
		self.prev_y = 0
		self.prev_noisy_x = 0
		self.prev_noisy_y = 0
		self.first_time = True
		self.noise_multiplier = 0.00001

	def odometry_cb(self, msg):
		#Get the x and y position of the robot in the simulation
		x = msg.pose.pose.position.x
		y = msg.pose.pose.position.y

		#Set the initial previous noisy x and y to the current x and y
		#if this is the first time the callback function has been called
		#Makes sure that the noisy odom and original odom have the same start
		#position. Otherwise, the position for noisy will start at zero
		if self.first_time:
			self.prev_noisy_x = x
			self.prev_noisy_y = y
			self.first_time = False

		#Determine how much the position of the robot has changed since
		#the last call of this function
		del_x = self.prev_x - x
		self.prev_x = x #Assign the previous value of x the current value of x
		del_y = self.prev_y - y
		self.prev_y = y

		#Apply a random number based on a normal distribution with the set deviation from a mean of 0
		noisy_del_x, noisy_del_y = noise.app_normal(del_x, del_y, (self.noise_multiplier*((del_x+del_y)/2)))

		#Keep everything in the original topic message
		odometry = msg

		#Determine the x and y positions given the noise.
		#Because we are adding the noise to the previous x and y, the noisy_odom
		#topic will always be one step behind the /bowser2/odom topic
		odometry.pose.pose.position.x = self.prev_noisy_x + noisy_del_x
		odometry.pose.pose.position.y = self.prev_noisy_y + noisy_del_y

		#Reassign the previous noisy z and y values
		self.prev_noisy_x = self.prev_noisy_x + noisy_del_x
		self.prev_noisy_y = self.prev_noisy_y + noisy_del_y

		#publish the data to the noisy_odom topic
		noise_pub.publish(odometry)


tpc = Topic_callbacks()
noise = Noise()

odometry = Odometry()
odometry.pose.pose.position.x = 0
odometry.pose.pose.position.y = 0

if __name__ == "__main__":
	rospy.init_node('my_odom') #Initialize a node of function argument name
	rospy.Subscriber('/bowser2/odom', Odometry, tpc.odometry_cb) #Subscribe to the robot's odometry topic
	noise_pub = rospy.Publisher('/bowser2/noisy_odom', Odometry, queue_size = 100)
	rospy.spin()

#mu = 0
#sigma = 0.1

#for _ in range(10):
#	print(rand.gauss(mu, sigma))
