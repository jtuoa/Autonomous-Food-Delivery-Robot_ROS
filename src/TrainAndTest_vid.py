import cv2
import numpy as np
import operator
import os

# module level variables ##########################################################################
MIN_CONTOUR_AREA = 300 #1220, 1500
MAX_CONTOUR_AREA = 1900 #1900, 2000
RESIZED_IMAGE_WIDTH = 20
RESIZED_IMAGE_HEIGHT = 30

###################################################################################################
class ContourWithData():

	# member variables ############################################################################
	npaContour = None           # contour
	boundingRect = None         # bounding rect for contour
	intRectX = 0                # bounding rect top left corner x location
	intRectY = 0                # bounding rect top left corner y location
	intRectWidth = 0            # bounding rect width
	intRectHeight = 0           # bounding rect height
	fltArea = 0.0               # area of contour

	def calculateRectTopLeftPointAndWidthAndHeight(self):               # calculate bounding rect info
		  [intX, intY, intWidth, intHeight] = self.boundingRect
		  self.intRectX = intX
		  self.intRectY = intY
		  self.intRectWidth = intWidth
		  self.intRectHeight = intHeight

	def checkIfContourIsValid(self):                            # this is oversimplified, for a production grade program
		  if self.fltArea < MIN_CONTOUR_AREA or self.fltArea > MAX_CONTOUR_AREA: return False        # much better validity checking would be necessary
		  return True

###################################################################################################

cap = cv2.VideoCapture(1)

if cap.isOpened() == False:
	print ("Unable to read camera feed")

while(cap.isOpened()):
	#capture frame by frame
	ret, frame = cap.read()
	if ret == True:
		allContoursWithData = [] # declare empty lists,
		validContoursWithData = [] # we will fill these shortly
	
		npaClassifications = np.loadtxt("classifications.txt", np.float32) #read in class images
		npaFlattenedImages = np.loadtxt("flattened_images.txt", np.float32) # read in training images

		npaClassifications = npaClassifications.reshape((npaClassifications.size, 1)) # reshape numpy array to 1d, necessary to pass to call to train

		kNearest = cv2.KNearest() # instantiate KNN object

		kNearest.train(npaFlattenedImages, npaClassifications)

		imgTestingNumbers = frame # cv2.imread("test5.png") read in testing numbers image

		imgGray = cv2.cvtColor(imgTestingNumbers, cv2.COLOR_BGR2GRAY) # get grayscale image
		imgBlurred = cv2.GaussianBlur(imgGray, (5,5), 0) # blur

		# filter image from grayscale to black and white
		imgThresh = cv2.adaptiveThreshold(imgBlurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)                                    

		imgThreshCopy = imgThresh.copy()

		npaContours, npaHierarchy = cv2.findContours(imgThreshCopy, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
		for npaContour in npaContours:                             # for each contour
			contourWithData = ContourWithData()                                             # instantiate a contour with data object
			contourWithData.npaContour = npaContour                                         # assign contour to contour with data
			contourWithData.boundingRect = cv2.boundingRect(contourWithData.npaContour)     # get the bounding rect
			contourWithData.calculateRectTopLeftPointAndWidthAndHeight()                    # get bounding rect info
			contourWithData.fltArea = cv2.contourArea(contourWithData.npaContour)           # calculate the contour area
			allContoursWithData.append(contourWithData)                                     # add contour with data object to list of all contours with data
			# end for

		for contourWithData in allContoursWithData:                 # for all contours
			if contourWithData.checkIfContourIsValid():             # check if valid
					validContoursWithData.append(contourWithData)       # if so, append to valid contour list
			# end if
			# end for

		validContoursWithData.sort(key = operator.attrgetter("intRectX"))         # sort contours from left to right

		strFinalString = ""         # declare final string, this will have the final number sequence by the end of the program

		for contourWithData in validContoursWithData:            # for each contour
						                                  # draw a green rect around the current char
			cv2.rectangle(imgTestingNumbers,                                        # draw rectangle on original testing image
						        (contourWithData.intRectX, contourWithData.intRectY),     # upper left corner
						        (contourWithData.intRectX + contourWithData.intRectWidth, contourWithData.intRectY + contourWithData.intRectHeight),      # lower right corner
						        (0, 255, 0),              # green
						        2)                        # thickness

			imgROI = imgThresh[contourWithData.intRectY : contourWithData.intRectY + contourWithData.intRectHeight,     # crop char out of threshold image
						             contourWithData.intRectX : contourWithData.intRectX + contourWithData.intRectWidth]

			imgROIResized = cv2.resize(imgROI, (RESIZED_IMAGE_WIDTH, RESIZED_IMAGE_HEIGHT))             # resize image, this will be more consistent for recognition and storage

			npaROIResized = imgROIResized.reshape((1, RESIZED_IMAGE_WIDTH * RESIZED_IMAGE_HEIGHT))      # flatten image into 1d numpy array

			npaROIResized = np.float32(npaROIResized)       # convert from 1d numpy array of ints to 1d numpy array of floats

			retval, npaResults, neigh_resp, dists = kNearest.find_nearest(npaROIResized, k = 1)     # call KNN function find_nearest

			strCurrentChar = str(chr(int(npaResults[0][0])))                                             # get character from results

			strFinalString = strFinalString + strCurrentChar            # append current char to full string
		# end for

		print "\n" + strFinalString + "\n"                  # show the full string

		cv2.imshow("imgTestingNumbers", imgTestingNumbers)      # show input image with green boxes drawn around found digits
		if cv2.waitKey(1) & 0Xff == ord('q'):
			break
			
	else:
		break
                                

cap.release()
cv2.destroyAllWindows()             # remove windows from memory


	
	




