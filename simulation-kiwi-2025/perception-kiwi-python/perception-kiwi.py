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
distances = { "front": 10.0, "left": 10.0, "right": 10.0, "rear": 10.0 };

d_thresh   = 0.30   # [m] distance at which to trigger avoidance
t_avoid    = 0.7    # [s] time to spend in AVOID
t_search   = 3.0    # [s] optional search timeout
s_thresh   = 0.9    # fraction of image width/height to consider on top of target
max_speed  = 0.15   # forward speed in WANDER & APPROACH
max_turn   = 0.4    # rad/s  turn speed

# --- FSM state variables ---
state        = "WANDER"
state_entry  = time.time()
turn_dir     = 1       # +1 = left, -1 = right
target_seen  = False

################################################################################
# This callback is triggered whenever there is a new distance reading coming in.
def onDistance(msg, senderStamp, timeStamps):
    print ("Received distance; senderStamp= %s" % (str(senderStamp)))
    print ("sent: %s, received: %s, sample time stamps: %s" % (str(timeStamps[0]), str(timeStamps[1]), str(timeStamps[2])))
    print ("%s" % (msg))
    if senderStamp == 0:
        distances["front"] = msg.distance
    if senderStamp == 1:
        distances["left"] = msg.distance
    if senderStamp == 2:
        distances["rear"] = msg.distance
    if senderStamp == 3:
        distances["right"] = msg.distance


# Create a session to send and receive messages from a running OD4Session;
def send_control(steering, speed):
    # steering: radians (left positive), speed: pedal position
    g = opendlv_message_standard_pb2.opendlv_proxy_GroundSteeringRequest()
    p = opendlv_message_standard_pb2.opendlv_proxy_PedalPositionRequest()
    g.groundSteering = steering
    p.position       = speed
    session.send(1090, g.SerializeToString())
    session.send(1086, p.SerializeToString())


# Replay mode: CID = 253
# Live mode: CID = 112
# TODO: Change to CID 112 when this program is used on Kiwi.
session = OD4Session.OD4Session(cid=111)
# Register a handler for a message; the following example is listening
# for messageID 1039 which represents opendlv.proxy.DistanceReading.
# Cf. here: https://github.com/chalmers-revere/opendlv.standard-message-set/blob/master/opendlv.odvd#L113-L115
messageIDDistanceReading = 1039
session.registerMessageCallback(messageIDDistanceReading, onDistance, opendlv_message_standard_pb2.opendlv_proxy_DistanceReading)
# Connect to the network session.
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

    # Turn buf into img array (1280 * 720 * 4 bytes (ARGB)) to be used with OpenCV.
    # img = numpy.frombuffer(buf, numpy.uint8).reshape(720, 1280, 4)
    
    # Turn buf into img array (1280 * 720 * 4 bytes (BGR)) to be used with OpenCV.
    img = numpy.frombuffer(buf, numpy.uint8).reshape(720, 1280, 3)

    ############################################################################
    # TODO: Add some image processing logic here.
 
    # Invert colors
    # img = cv2.bitwise_not(img)

    # Draw a red rectangle
    # cv2.rectangle(img, (50, 50), (100, 100), (0,0,255), 2)

    # TODO: Disable the following two lines before running on Kiwi:
    cv2.imshow("image", img);
    cv2.waitKey(2);

    now = time.time()
    front = distances["front"]
    
        # Optionally detect blue target in img:
    # (simple HSV thresholding, find largest contour, set target_seen True+store its center)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (100,150,50), (130,255,255))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        # pick largest
        c = max(contours, key=cv2.contourArea)
        x,y,w,h = cv2.boundingRect(c)
        frac = max(w/img.shape[1], h/img.shape[0])
        if frac > 0.02:  # small noise filter
            target_seen = True
            tx = x + w/2 - img.shape[1]/2

    # FSM transitions & actions
    if state == "WANDER":
        if front < d_thresh:
            state = "AVOID"; state_entry = now
            turn_dir = random.choice([-1,1])
        elif target_seen:
            state = "APPROACH"; state_entry = now
        else:
            # slight random wiggle
            steer = 0.0 + random.uniform(-0.1, 0.1)
            send_control(steer, max_speed)

    elif state == "AVOID":
        if now - state_entry < t_avoid:
            # back up and turn
            send_control(turn_dir * max_turn, -0.1)
        else:
            state = "WANDER"; state_entry = now

    elif state == "APPROACH":
        if not target_seen:
            # lost sight  go search or just wander
            state = "WANDER"; state_entry = now
        elif frac > s_thresh:
            state = "PARK"; state_entry = now
        else:
            # center target: steer proportional to tx
            steer = -0.002 * tx  
            send_control(steer, max_speed)

    elif state == "PARK":
        send_control(0.0, 0.0)
        
    ############################################################################
    # Example: Accessing the distance readings.
    print ("Front = %s" % (str(distances["front"])))
    print ("Left = %s" % (str(distances["left"])))
    print ("Right = %s" % (str(distances["right"])))
    print ("Rear = %s" % (str(distances["rear"])))

    ############################################################################
    # Example for creating and sending a message to other microservices; can
    # be removed when not needed.
    angleReading = opendlv_message_standard_pb2.opendlv_proxy_AngleReading()
    angleReading.angle = 123.45

    # 1038 is the message ID for opendlv.proxy.AngleReading
    session.send(1038, angleReading.SerializeToString());

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

