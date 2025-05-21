#!/usr/bin/env python3

# Copyright (C) 2024 OpenDLV

# sysv_ipc is needed to access the shared memory where the camera image is present.
import sysv_ipc
# numpy and cv2 are needed to access, modify, or display the pixels
import numpy
import cv2
# OD4Session is needed to send and receive messages
import OD4Session
# Import the OpenDLV messages.
import opendlv_message_standard_pb2
import time, random

################################################################################
# This dictionary contains all distance values to be filled by function onDistance(...).
distances = { "front": 0,
              "left_front": 0,
              "left_rear":0,
              "right_front": 0,
              "right_rear":0,
              "rear": 0,
              "sonic_distance":0
              };

front_distance = 0
SAFE_DISTANCE = 0.1
update_interval = 0.1
d_thresh = 0.1
t_avoid = 0.7

# --- FSM state variables ---

STATES = {
    "EXPLORE": 0,
    "AVOID":1,
    "APPROACH": 2,
    "STOP": 3
}

current_state = STATES["EXPLORE"]
# avoid_counter = 0
TARGET_SIZE = 75  # mm
got_initial_reading = False

def detect_target(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
  
    lower_blue = numpy.array([100, 150, 50])
    upper_blue = numpy.array([140, 255, 255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    
    kernel = numpy.ones((5,5), numpy.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 500:
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = float(w)/h

            if 0.8 < aspect_ratio < 1.2 and area > 500:
                return (x + w//2, y + h//2), (w, h)
    return None, None
    
def update_fsm():
    pass

################################################################################
# This callback is triggered whenever there is a new distance reading coming in.
def onDistance(msg, senderStamp, timeStamps):
    print ("Received distance; senderStamp= %s" % (str(senderStamp)))
    print ("sent: %s, received: %s, sample time stamps: %s" % (str(timeStamps[0]), str(timeStamps[1]), str(timeStamps[2])))
    print ("%s" % (msg))
    global got_initial_reading
    if senderStamp == 5:
        distances["front"] = 0.13 / msg.voltage
    elif senderStamp == 4:
        distances["left_front"] = 0.13 / msg.voltage
    elif senderStamp == 0:
        distances["left_rear"] = 0.13 / msg.voltage
    elif senderStamp == 6:
        distances["rear"] = 0.13 / msg.voltage
    elif senderStamp == 2:
        distances["right_rear"] = 0.13 / msg.voltage
    elif senderStamp == 1:
        distances["right_front"] = 0.13 / msg.voltage
    elif senderStamp == 7:
        distances["sonic_distance"] = msg.distance
        
    got_initial_reading = True
    
#def frontDistance(msg, senderStamp, timeStamp):
#    print ("Received distance; senderStamp= %s" % (str(senderStamp)))
#    print ("%s" % (msg))
#    global got_initial_reading
#    if senderStamp == 7:
#        front_distance = msg.distance
#    got_initial_reading = True

# Create a session to send and receive messages from a running OD4Session;
def send_control(left_pedal, right_pedal):
    
    lppr = opendlv_message_standard_pb2.opendlv_proxy_PedalPositionRequest()
    lppr.position = left_pedal
    session.send(1086, lppr.SerializeToString(), senderStamp=0)
    
    rppr = opendlv_message_standard_pb2.opendlv_proxy_PedalPositionRequest()
    rppr.position = right_pedal
    session.send(1086, rppr.SerializeToString(), senderStamp=1)


# Replay mode: CID = 253
# Live mode: CID = 112
# TODO: Change to CID 112 when this program is used on Kiwi.
session = OD4Session.OD4Session(cid=111)
# Register a handler for a message; the following example is listening
# for messageID 1039 which represents opendlv.proxy.DistanceReading.
# Cf. here: https://github.com/chalmers-revere/opendlv.standard-message-set/blob/master/opendlv.odvd#L113-L115
messageIDDistanceReading = 1039
session.registerMessageCallback(messageIDDistanceReading, onDistance, opendlv_message_standard_pb2.opendlv_proxy_DistanceReading)

messageIDVoltageReading = 1037
session.registerMessageCallback(messageIDVoltageReading, onDistance,
opendlv_message_standard_pb2.opendlv_proxy_VoltageReading)

# connect to the network session.
session.connect()

################################################################################
# The following lines connect to the camera frame that resides in shared memory.
# This name must match with the name used in the h264-decoder-viewer.yml file.
name = "/tmp/video0.bgr"
# Obtain the keys for the shared memory and semaphores.
keySharedMemory = sysv_ipc.ftok(name, 1, True)
keySemMutex = sysv_ipc.ftok(name, 2, True)
keySemCondition = sysv_ipc.ftok(name, 3, True)
# Instantiate the SharedMemory and Semaphore objects.
shm = sysv_ipc.SharedMemory(keySharedMemory)
mutex = sysv_ipc.Semaphore(keySemCondition)
cond = sysv_ipc.Semaphore(keySemCondition)

################################################################################
# Main loop to process the next image frame coming in.
while True:
    # Wait for next notification.
    cond.Z()
    print ("Received new frame.")

    # Lock access to shared memory.
    mutex.acquire()
    # Attach to shared memory.
    shm.attach()
    # Read shared memory into own buffer.
    buf = shm.read()
    # Detach to shared memory.
    shm.detach()
    # Unlock access to shared memory.
    mutex.release()
    
    t0 = time.time()
    update_fsm()

    # Turn buf into img array (1280 * 720 * 4 bytes (ARGB)) to be used with OpenCV.
    # img = numpy.frombuffer(buf, numpy.uint8).reshape(720, 1280, 4)
    
    # Turn buf into img array (1280 * 720 * 4 bytes (BGR)) to be used with OpenCV.
    img = numpy.frombuffer(buf, numpy.uint8).reshape(720, 1280, 3)

    ############################################################################
    # TODO: Add some image processing logic here.
    
    # session.registerMessageCallback(messageIDDistanceReading, onDistance, opendlv_message_standard_pb2.opendlv_proxy_DistanceReading)
    
    target_center, target_size = detect_target(img)
    target_detected = (target_center is not None)

    if current_state == STATES["STOP"]:
        pass

    elif target_detected and current_state != STATES["STOP"]:
        current_state = STATES["APPROACH"]
    
    elif current_state == STATES["APPROACH"]:
        if not target_detected:
            current_state = STATES["EXPLORE"]
        else:
            img_center = img.shape[1] // 2
            pos_threshold = 50
        
            if (abs(target_center[0] - img_center) < pos_threshold
                and target_size[0] > 100
                and target_size[1] > 100):
                current_state = STATES["STOP"]

    elif current_state == STATES["AVOID"]:
        # avoid_counter -= 1
        if distances["sonic_distance"] > SAFE_DISTANCE and distances["left_front"]>SAFE_DISTANCE and distances["right_front"]>SAFE_DISTANCE : # avoid_counter <= 0:
            current_state = STATES["EXPLORE"]

    elif current_state == STATES["EXPLORE"]:
        if got_initial_reading and distances["sonic_distance"] < SAFE_DISTANCE:
            current_state = STATES["AVOID"]
        elif distances["right_front"]<SAFE_DISTANCE:
            current_state = STATES["AVOID"]
        elif distances["left_front"]<SAFE_DISTANCE:
            current_state = STATES["AVOID"]
            # avoid_counter = 15
	

    if current_state != STATES["STOP"]:
        if current_state == STATES["EXPLORE"]:
            left_pedal = 0.1
            right_pedal = 0.1
        
        elif current_state == STATES["AVOID"]:

            if distances["left_front"] > distances["right_front"]:
                left_pedal = -0.1
                right_pedal = 0.1
            else:  #if distances["left_front"] < distances["right_front"]:
                left_pedal = 0.1
                right_pedal = -0.1
                
            
        elif current_state == STATES["APPROACH"]:
            left_pedal = 0.1
            right_pedal = 0.1
	
        # img_center = img.shape[1] // 2
        # error = target_center[0] - img_center
        # steering_angle = -error * 0.005
        
        else:
           left_pedal = 0.0
           right_pedal = 0.0
        
    send_control(left_pedal, right_pedal)
    if current_state == STATES["AVOID"]:
        time.sleep (0.5)
    
    # Optional: overlay state and detection for debugging
    state_names = {v:k for k,v in STATES.items()}
    cv2.putText(img, f"State: {state_names[current_state]}", (10,30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
    if target_detected:
        cv2.circle(img, target_center, 10, (0,255,0), -1)
        
    elapsed = time.time() - t0
    time.sleep(max(0,update_interval - elapsed))

    # During simulation only:
    cv2.imshow("Kiwi view", img)
    cv2.waitKey(2)
        

    #groundSteeringRequest = opendlv_message_standard_pb2.opendlv_proxy_GroundSteeringRequest()
    #groundSteeringRequest.groundSteering = steering_angle
    #session.send(1090, groundSteeringRequest.SerializeToString())

    #pedalPositionRequest = opendlv_message_standard_pb2.opendlv_proxy_PedalPositionRequest()
    #pedalPositionRequest.position = pedal_speed
    #session.send(1086, pedalPositionRequest.SerializeToString())
        
    ############################################################################
    # Example: Accessing the distance readings.
    print ("Front = %s" % (str(distances["front"])))
    print ("Left_front = %s" % (str(distances["left_front"])))
    print ("Left_rear = %s" % (str(distances["left_rear"])))
    print ("Right_front = %s" % (str(distances["right_front"])))
    print ("Right_rear = %s" % (str(distances["right_rear"])))
    print ("Rear = %s" % (str(distances["rear"])))
    print ("Ultrasonic_distance = %s" % (str(distances["sonic_distance"])))

    ############################################################################
    # Example for creating and sending a message to other microservices; can
    # be removed when not needed.
    # angleReading = opendlv_message_standard_pb2.opendlv_proxy_AngleReading()
    # angleReading.angle = 123.45

    # 1038 is the message ID for opendlv.proxy.AngleReading
    # session.send(1038, angleReading.SerializeToString());

    ############################################################################
    # Steering and acceleration/decelration.
    #
    # Uncomment the following lines to steer; range: +38deg (left) .. -38deg (right).
    # Value groundSteeringRequest.groundSteering must be given in radians (DEG/180. * PI).
    #groundSteeringRequest = opendlv_message_standard_pb2.opendlv_proxy_GroundSteeringRequest()
    #groundSteeringRequest.groundSteering = 0
    #session.send(1090, groundSteeringRequest.SerializeToString());

    # Uncomment the following lines to accelerate/decelerate; range: +0.25 (forward) .. -1.0 (backwards).
    # Be careful!
    #pedalPositionRequest = opendlv_message_standard_pb2.opendlv_proxy_PedalPositionRequest()
    #pedalPositionRequest.position = 0
    #session.send(1086, pedalPositionRequest.SerializeToString());

