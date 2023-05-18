# import the necessary packages
import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray
import face_recognition
import imutils
from imutils.video import FPS
from imutils.video import VideoStream
from imutils import paths
import pickle
import time

#import os library that is used to creare new folder for every new face entry
import os

#define functions
def trainmodel():

    # our images are located in the dataset folder
    print("[INFO] start processing faces...")
    imagePaths = list(paths.list_images("DataSet"))

    # initialize the list of known encodings and known names
    knownEncodings = []
    knownNames = []

    # loop over the image paths
    for (i, imagePath) in enumerate(imagePaths):
        # extract the person name from the image path
        print("[INFO] processing image {}/{}".format(i + 1,
            len(imagePaths)))
        name = imagePath.split(os.path.sep)[-2]

        # load the input image and convert it from RGB (OpenCV ordering)
        # to dlib ordering (RGB)
        image = cv2.imread(imagePath)
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # detect the (x, y)-coordinates of the bounding boxes
        # corresponding to each face in the input image
        boxes = face_recognition.face_locations(rgb,
            model="hog")

        # compute the facial embedding for the face
        encodings = face_recognition.face_encodings(rgb, boxes)

        # loop over the encodings
        for encoding in encodings:
            # add each encoding + name to our set of known names and
            # encodings
            knownEncodings.append(encoding)
            knownNames.append(name)

    # dump the facial encodings + names to disk
    print("[INFO] serializing encodings...")
    data = {"encodings": knownEncodings, "names": knownNames}
    f = open("encodings.pickle", "wb")
    f.write(pickle.dumps(data))
    f.close()

def AddNewFace():

    #Ask the user to enter the name of the person
    name = str(input("Please enter the name to register new face: "))

    #make a new directry in Dataset with entered name
    if os.path.exists("DataSet/"+ name) == False:
        os.mkdir("DataSet/"+ name)

    #Set up the camera
    camera = PiCamera()
    camera.resolution = (512, 304)
    camera.framerate = 10
    rawCapture = PiRGBArray(camera, size=(512, 304))

    #set the image count to 10
    img_counter = 10

    #Start the loop so the user can capture more than 1 picture at the same time
    while img_counter != 0:

        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

            image = frame.array
            cv2.imshow("Press Space to take a photo", image)
            rawCapture.truncate(0)

            k = cv2.waitKey(1)
            rawCapture.truncate(0)

            #exit if user press esc
            if img_counter == 0:
                print("picture capturing Complete, closing...")
                break
            #capture image when user press space
            elif k%256 == 32:
                # SPACE pressed

                #get the name for the Image
                img_name = "DataSet/"+ name +"/image_{}.jpg".format(10-img_counter)
                cv2.imwrite(img_name, image)
                print("{} written!".format(img_name))
                img_counter -= 1

        #exit if user press esc
        if k%256 == 27:
            print("Escape hit, closing...")
        break

    cv2.destroyAllWindows()
    trainmodel()


#Ask if the user wants to add a new face
if 'y' == str(input("Press y if you want to add a new face: ")):
    AddNewFace()
