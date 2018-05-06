#!/usr/bin/env python

import rospy, cv2, cv_bridge
import actionlib
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import operator
import os
from playsound import playsound

MIN_CONTOUR_AREA = 1220 # 300 (BDAY) 1220, 1500 (T)
MAX_CONTOUR_AREA = 1900 #1900, 2000
RESIZED_IMAGE_WIDTH = 20
RESIZED_IMAGE_HEIGHT = 30

waypoints = [[(4.087, 0.717, 0.0), (0.0, 0.0, 0.995, 0.096)],
		[(1.158, -3.943, 0.0), (0.0, 0.0, -0.783, 0.622)],
		[(1.109, 0.084, 0.0), (0.0, 0.0, 0.986, 0.168)],
		[(2.652, 3.483, 0.0), (0.0, 0.0, 0.584, 0.812)]
]

###################################################################################################
class ContourWithData(object):
	def __init__(self):
		self.boundingRect = None # bounding rect for contour
		self.intRectX = 0 # bounding rect top left corner x location
		self.intRectY = 0 # bounding rect top left corner y location
		self.intRectWidth = 0 # bounding rect width
		self.intRectHeight = 0 # bounding rect height
		self.fltArea = 0.0 # area of contour

	def calculateRectTopLeftPointAndWidthAndHeight(self): # calculate bounding rect info
		  [intX, intY, intWidth, intHeight] = self.boundingRect
		  self.intRectX = intX
		  self.intRectY = intY
		  self.intRectWidth = intWidth
		  self.intRectHeight = intHeight

	def checkIfContourIsValid(self): #check valid contour via area
		  if self.fltArea < MIN_CONTOUR_AREA or self.fltArea > MAX_CONTOUR_AREA: return False
		  return True

###################################################################################################

class Comp5(object):
	def __init__(self):
		rospy.Subscriber('camera/rgb/camera_info', CameraInfo, self.caminfo_cb)
		#rospy.Subscriber('camera/rgb/image_raw', Image, self.image_cb)
		rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_cb)
				
		#self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 1)
		self.cmd_vel_pub = rospy.Publisher('cmd_vel/velocityramp', Twist, queue_size=1)
		self.twist = Twist()

		self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)  
		self.client.wait_for_server()
		
		self.fpv_vid()
		
		
	def caminfo_cb(self, msg):
		self.K = np.array(msg.K).reshape(3,3)
		self.D = np.array(msg.D)

	#get bot current pos 
	def amcl_cb(self, msg):
		self.pose = msg.pose.pose
		self.px = self.pose.position.x
		self.py = self.pose.position.y
		print self.px, self.py
					
	def makeSound(self, i):
		if (i==1):
			playsound('bday.wav')
		
	#turn waypoints into a movebase goal
	def goal_pose(self,pose):
		goal_pose = MoveBaseGoal()
		goal_pose.target_pose.header.frame_id = 'map'
		goal_pose.target_pose.pose.position.x = pose[0][0]
		goal_pose.target_pose.pose.position.y = pose[0][1]
		goal_pose.target_pose.pose.position.z = pose[0][2]
		goal_pose.target_pose.pose.orientation.x = pose[1][0]
		goal_pose.target_pose.pose.orientation.y = pose[1][1]
		goal_pose.target_pose.pose.orientation.z = pose[1][2]
		goal_pose.target_pose.pose.orientation.w = pose[1][3]
		return goal_pose
		
	def fpv_vid(self):
		cap = cv2.VideoCapture(0) #init vid
		if cap.isOpened() == False:
			print ("Unable to read camera feed")

		while(cap.isOpened()):
			#capture frame by frame
			ret, frame = cap.read()
			if ret == True:
				allContoursWithData = [] 
				validContoursWithData = [] 
	
				npaClassifications = np.loadtxt("classifications.txt", np.float32) #read in class img
				npaFlattenedImages = np.loadtxt("flattened_images.txt", np.float32) # read in training img

				npaClassifications = npaClassifications.reshape((npaClassifications.size, 1)) # reshape 

				kNearest = cv2.KNearest() # instantiate KNN object

				kNearest.train(npaFlattenedImages, npaClassifications)

				imgTestingNumbers = frame # cv2.imread("test5.png") read in testing numbers image

				imgGray = cv2.cvtColor(imgTestingNumbers, cv2.COLOR_BGR2GRAY)
				imgBlurred = cv2.GaussianBlur(imgGray, (5,5), 0) # blur

				# filter image from grayscale to black and white
				imgThresh = cv2.adaptiveThreshold(imgBlurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)                                    

				imgThreshCopy = imgThresh.copy()

				npaContours, npaHierarchy = cv2.findContours(imgThreshCopy, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
				
				for npaContour in npaContours: # for each contour
					contourWithData = ContourWithData()
					contourWithData.npaContour = npaContour
					contourWithData.boundingRect = cv2.boundingRect(contourWithData.npaContour)
					contourWithData.calculateRectTopLeftPointAndWidthAndHeight()
					contourWithData.fltArea = cv2.contourArea(contourWithData.npaContour)
					allContoursWithData.append(contourWithData)
					# end for

				for contourWithData in allContoursWithData: # for all contours
					if contourWithData.checkIfContourIsValid(): # check if valid
							validContoursWithData.append(contourWithData) 
					# end if
					# end for

				validContoursWithData.sort(key = operator.attrgetter("intRectX"))

				strFinalString = ""

				for contourWithData in validContoursWithData:

					cv2.rectangle(imgTestingNumbers, (contourWithData.intRectX, contourWithData.intRectY), (contourWithData.intRectX + contourWithData.intRectWidth, contourWithData.intRectY + contourWithData.intRectHeight), (0, 255, 0), 2)                        

					imgROI = imgThresh[contourWithData.intRectY : contourWithData.intRectY + contourWithData.intRectHeight, contourWithData.intRectX : contourWithData.intRectX + contourWithData.intRectWidth]

					imgROIResized = cv2.resize(imgROI, (RESIZED_IMAGE_WIDTH, RESIZED_IMAGE_HEIGHT))

					npaROIResized = imgROIResized.reshape((1, RESIZED_IMAGE_WIDTH * RESIZED_IMAGE_HEIGHT)) 

					npaROIResized = np.float32(npaROIResized)

					retval, npaResults, neigh_resp, dists = kNearest.find_nearest(npaROIResized, k = 1)

					strCurrentChar = str(chr(int(npaResults[0][0])))

					strFinalString = strFinalString + strCurrentChar
				# end for

				#print "\n" + strFinalString + "\n" # show the full string
				
				if strFinalString == "T1":
					wp_pose = waypoints[1] #POS 1
					wp_goal = self.goal_pose(wp_pose) #create movebase goal
					self.client.send_goal(wp_goal)
					self.client.wait_for_result()
					
					wp_pose = waypoints[0] #BACK HOME
					wp_goal = self.goal_pose(wp_pose) #create movebase goal
					self.client.send_goal(wp_goal)
					self.client.wait_for_result()
							
				elif strFinalString == "T2":
					wp_pose = waypoints[2] #POS 2
					wp_goal = self.goal_pose(wp_pose) #create movebase goal
					self.client.send_goal(wp_goal)
					self.client.wait_for_result()
					
					wp_pose = waypoints[0] #BACK HOME
					wp_goal = self.goal_pose(wp_pose) #create movebase goal
					self.client.send_goal(wp_goal)
					self.client.wait_for_result()
					
				elif strFinalString == "T3":
					wp_pose = waypoints[3] #POS 3
					wp_goal = self.goal_pose(wp_pose) #create movebase goal
					self.client.send_goal(wp_goal)
					self.client.wait_for_result()
					
					wp_pose = waypoints[0] #BACK HOME
					wp_goal = self.goal_pose(wp_pose) #create movebase goal
					self.client.send_goal(wp_goal)
					self.client.wait_for_result()
				
				elif strFinalString == "BD2":
					wp_pose = waypoints[2] #POS 1
					wp_goal = self.goal_pose(wp_pose) #create movebase goal
					self.client.send_goal(wp_goal)
					self.client.wait_for_result()
					
					self.makeSound(1) #play music 
					
					wp_pose = waypoints[0] #BACK HOME
					wp_goal = self.goal_pose(wp_pose) #create movebase goal
					self.client.send_goal(wp_goal)
					self.client.wait_for_result()
				
				#cv2.imshow("imgTestingNumbers", imgTestingNumbers) 
				if cv2.waitKey(1) & 0Xff == ord('q'):
					break
			
			else:
				break
				                            

		cap.release()
		cv2.destroyAllWindows() 

###################################################################################################
if __name__ == "__main__":
	rospy.init_node('comp5')
	comp5 = Comp5()
	rospy.spin()
	




