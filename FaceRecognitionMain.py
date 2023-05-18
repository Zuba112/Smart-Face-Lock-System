from imutils.video import VideoStream
from imutils.video import FPS
import RPi.GPIO as GPIO
from DetectionLib import *
detect = DetectionLib()
from MotorControl import *
Motor = MotorControl()

# initialize the video stream and allow the camera sensor to warm up
print("[INFO] starting video stream...")
#vs = VideoStream(src=0).start()
vs = VideoStream(usePiCamera=True).start()
time.sleep(2.0)

# start the FPS counter
fps = FPS().start()

Liveliness_Reset = 0
Face_Reset = 0

#Declare variable to save the Lock state
IsLocked = True

#Dcclare the interrupt buttons to lock and unlock
LOCKBUTTON_GPIO = 7
UNLOCKBUTTON_GPIO = 8

#Setup the pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(LOCKBUTTON_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(UNLOCKBUTTON_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)

#Define callback functions
def lock_button_pressed_callback(channel):
    global IsLocked
    if IsLocked == False:
        print("Motor is Starting to lock the door")
        Motor.lock()
        IsLocked = True
    
def unlock_button_pressed_callback(channel):
    global IsLocked
    if IsLocked == True:
        print("Motor is Starting to unlock the door")
        Motor.unlock()
        IsLocked = False
        
#Add interrupt callbacks
GPIO.add_event_detect(LOCKBUTTON_GPIO, GPIO.FALLING, 
            callback=lock_button_pressed_callback, bouncetime=100)

GPIO.add_event_detect(UNLOCKBUTTON_GPIO, GPIO.FALLING, 
            callback=unlock_button_pressed_callback, bouncetime=100)

# loop over frames from the video file stream
while True:
        
        #reset names and status every 20 elpsed f
        if Liveliness_Reset > 20:
            detect.Reset_Liveliness()
            Liveliness_Reset = 0
            
        #reset names every 120 elpsed f
        if Face_Reset > 120:
            detect.Reset_Names()
            Face_Reset = 0
        # grab the frame from the threaded video stream and resize it
        # to speedup processing)
        #src,frame = vs.read()
        frame = vs.read()
        #Get the mean of the current frame
        
        [DetectionFlag, newframe] = detect.processframe(frame, 300)
        cv2.imshow("Frame", newframe)
        
        #Check if detected faces is a recognized face and if it is unlock the door
        if DetectionFlag == True and IsLocked == True:
            print("Motor is Starting to unlock the door")
            Motor.unlock()
            IsLocked = False
        
        key = cv2.waitKey(1) & 0xFF
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
                break
        # update the FPS counter
        fps.update()
        Liveliness_Reset +=1
        Face_Reset +=1

# stop the timer and display FPS information
fps.stop()
print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
# do a bit of cleanup
cv2.destroyAllWindows()