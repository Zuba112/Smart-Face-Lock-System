# import the necessary packages
from imutils.video import VideoStream
from imutils.video import FPS
import face_recognition
import argparse
import imutils
import pickle
import time
import cv2
import threading

#import libraries for the blink test
from scipy.spatial import distance as dist
from imutils.video import FileVideoStream
from imutils import face_utils
import numpy as np
import dlib
from imutils.face_utils import FaceAligner
from imutils.face_utils import rect_to_bb

from imutils import paths
import os


class DetectionLib:

        def __init__(self):
                #define frontal face model
                self.detectface = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")

                #define facial landmarks predictor
                self.detectlandmarks = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")

                #get encoding data
                self.data = pickle.loads(open("encodings.pickle", "rb").read())

                # define two constants, one for the eye aspect ratio to indicate blink and then a second constant for the number of consecutive
                self.EYE_AR_THRESH = 0.2
                self.EYE_AR_CONSEC_FRAMES = 0.5

                # grab the indexes of the facial landmarks for the left and right eye, respectively
                (self.lStart, self.lEnd) = face_utils.FACIAL_LANDMARKS_IDXS["left_eye"]
                (self.rStart, self.rEnd) = face_utils.FACIAL_LANDMARKS_IDXS["right_eye"]

                #define variables for name tags
                self.counters = [0,0,0,0,0]
                self.FaceStatus = ["Fake","Fake","Fake","Fake","Fake"]
                self.names = ["Unknown","Unknown","Unknown","Unknown","Unknown"]
                self.FaceDetected = False
                self.FaceRecognizedFlag = False
                
        def Reset_Liveliness(self):
                #reset all variables containing data
                self.counters = [0,0,0,0,0]
                self.FaceStatus = ["Fake","Fake","Fake","Fake","Fake"]
        
        def Reset_Names(self):
            self.names = ["Unknown","Unknown","Unknown","Unknown","Unknown"]
            self.FaceRecognizedFlag = False

        #function to calculate the aspect ratio of the eye
        def eye_aspect_ratio(self,eye):
                # compute the euclidean distances between the two sets of
                # vertical eye landmarks (x, y)-coordinates
                A = dist.euclidean(eye[1], eye[5])
                B = dist.euclidean(eye[2], eye[4])
                # compute the euclidean distance between the horizontal
                # eye landmark (x, y)-coordinates
                C = dist.euclidean(eye[0], eye[3])
                # compute the eye aspect ratio
                ear = (A + B) / (2.0 * C)
                # return the eye aspect ratio
                return ear

        #Function to calculate the area of both eyes
        def calculate_area(self):

                # extract the left and right eye coordinates, then use the coordinates to compute the eye aspect ratio for both eyes
                leftEye = self.shape[self.lStart:self.lEnd]
                rightEye = self.shape[self.rStart:self.rEnd]
                leftEAR = self.eye_aspect_ratio(leftEye)
                rightEAR = self.eye_aspect_ratio(rightEye)

                # compute the convex hull for the left and right eye, then
                # visualize each of the eyes
                leftEyeHull = cv2.convexHull(leftEye)
                rightEyeHull = cv2.convexHull(rightEye)
                cv2.drawContours(self.fr, [leftEyeHull], -1, (0, 255, 0), 1)
                cv2.drawContours(self.fr, [rightEyeHull], -1, (0, 255, 0), 1)

                # average the eye aspect ratio together for both eyes
                return ((leftEAR + rightEAR) / 2.0)

        def test_liveliness(self, rect, i):

                if(self.FaceStatus[i] == "Fake"):
                        #Check if the eyes are blinking
                        self.shape = self.detectlandmarks(self.gray, rect)
                        self.shape = face_utils.shape_to_np(self.shape)

                        # extract the left and right eye coordinates, then use the
                        # coordinates to compute the eye aspect ratio for both eyes
                        self.ear = self.calculate_area()

                        # threshold, and if so, increment the blink frame counter
                        if self.ear < self.EYE_AR_THRESH:
                                self.counters[i] += 1
                        # otherwise, the eye aspect ratio is not below the blink
                        # threshold
                        else:
                                # if the eyes were closed for a sufficient number of
                                # then increment the total number of blinks
                                if self.counters[i] >= self.EYE_AR_CONSEC_FRAMES:
                                        # update the list of names
                                        self.FaceStatus[i] = "Real"
                                        print("Real Face detected")

                                        # attempt to match each face in the input image to our known
                                        # encodings
                                # reset the eye frame counter
                                self.counters[i] = 0

        def Recognize_Face(self,encoding,i):

                matches = face_recognition.compare_faces(self.data["encodings"],encoding)
                best_match = np.argmin(matches)

                # check to see if we have found a match
                if matches[best_match]:
                        self.names[i] = self.data["names"][best_match]
                        print("Person recognized: %c", self.names[i])
                        self.FaceRecognizedFlag = True
                        

        #function to process each frame
        def processframe(self,frame, frame_width):

                #resize the frame to a suitable width
                self.fr = imutils.resize(frame, width=frame_width)

                #convert the file into greyscale
                self.gray = cv2.cvtColor(self.fr, cv2.COLOR_BGR2GRAY)
                
                #convert in rgb
                self.rgb = cv2.cvtColor(self.fr, cv2.COLOR_BGR2RGB)

                #detect faces from the frames
                self.detectfaces()

                #check if there is any detected face
                if len(self.rects) != 0:
                        FaceDetected = False
                        # Boxes contains (x,y, h, w) for each detected face
                        self.dlib_rects = [dlib.rectangle(x, y, x+h, y+w) for (x, y, h, w) in self.rects]

                        for (i,rect) in enumerate(self.dlib_rects):
                                threadProcess = threading.Thread(name='simplethread', target=self.test_liveliness, args=[rect, i])
                                threadProcess.daemon = True
                                threadProcess.start()
                                #self.test_liveliness(rect, i)

                        for (i,box) in enumerate(self.boxes):
                                if (self.FaceStatus[i] == "Real" and self.names[i] == "Unknown"):
                                    
                                    print("Start face recognition")
                                    
                                    encodings = face_recognition.face_encodings(self.rgb, self.boxes)
                                    
                                    threadProcess = threading.Thread(name='simplethread', target=self.Recognize_Face, args=[encodings[i], i])
                                    threadProcess.daemon = True
                                    threadProcess.start()
                                    
                                    # compute the facial embeddings for each face bounding box
                                    #matches = face_recognition.compare_faces(self.data["encodings"],encodings[i])
                                    #best_match = np.argmin(matches)

                                    # check to see if we have found a match
                                    #if matches[best_match]:
                                        #self.names[i] = self.data["names"][best_match]
                                    

                self.insertrectangles()
                return [self.FaceRecognizedFlag, self.fr]

        #Function to detect faces from the gray image using frontal face detection
        def detectfaces(self):

                # detect faces in the grayscale frame using the frontal face detector
                self.rects = self.detectface.detectMultiScale(self.gray, scaleFactor=1.1,
                minNeighbors=7, minSize=(30, 30))

                #convert rectangle into left right up and down
                self.boxes = [(y, x + w, y + h, x) for (x, y, w, h) in self.rects]


        def insertrectangles(self):

                for (i,(top, right, bottom, left)) in enumerate(self.boxes):
                        cv2.rectangle(self.fr, (left, top), (right, bottom),
                                (0, 255, 0), 2)
                        y = top - 5 if top - 5 > 5 else top + 5
                        cv2.putText(self.fr, self.names[i], (left, y), cv2.FONT_HERSHEY_SIMPLEX,
                                0.4, (0, 255, 0), 2)
                        y = bottom + 12 if bottom + 12 > 12 else bottom - 12
                        cv2.putText(self.fr, self.FaceStatus[i], (left, y), cv2.FONT_HERSHEY_SIMPLEX,
                                0.4, (0, 255, 0), 2)
