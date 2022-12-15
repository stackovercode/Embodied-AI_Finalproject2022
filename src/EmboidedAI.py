#!/usr/bin/env python3
import ev3dev.ev3 as ev3
import time
import signal
import sys

#                           gyro - negative value
#                           light sensor - input 4
#                                 left turn            
#                                       
#
#                                   ------ 
#                                  |      |
#                 ------------------------
#   Motor        |                        |         Motor
#   negative     |                        |         Positive values 
#    Back <-     |                        |         -> Front
#                |                        |
#                |                        |
#                |                        |
#                 ------------------------
#                                  |      |
#                                   ------   
#
#                                right turn            
#                           gyro - positive value
#                           light sensor - input 1

# Sensors
cs_left = ev3.ColorSensor("in1")
cs_right = ev3.ColorSensor("in4")
gs = ev3.GyroSensor()
echoSens = ev3.UltrasonicSensor()

# Actuators
mLeft = ev3.LargeMotor("outD")
mRight = ev3.LargeMotor("outA")
gripper = ev3.Motor("outB")

cs_left.mode  = 'COL-REFLECT'
cs_right.mode = 'COL-REFLECT'

MAX_SPEED = 500
# -------------------- FUNCTIONS --------------------
def saturateSpeed(left_speed, right_speed):
    '''
    Function for limiting the commanded speed.
    The EV3 motors throw an exception if the absolute commanded speed is above 900.
    '''
    # Lower cap
    left_speed = max(left_speed, -MAX_SPEED)
    right_speed = max(right_speed, -MAX_SPEED)
    
    # Upper cap
    left_speed = min(left_speed, MAX_SPEED)
    right_speed = min(right_speed, MAX_SPEED)
    
    return left_speed, right_speed

def signal_handler(sig, frame):
    '''
    Signal handler. Detects program interrupts and stops motors before halting program.
    '''
    
    print('Stopping motors and shutting down')
    mLeft.stop(stop_action="coast")
    mRight.stop(stop_action="coast")
    gripper.stop(stop_action="coast")
    time.sleep(1)
    sys.exit(0)    

# Sensor offsets
gs_offset = gs.value()

# Line follow
offsetLeft = (cs_left.value() + 0) / 2.0 # Left sensor offset
offsetRight = (cs_right.value() + 0 ) / 2.0 # Right sensor offset
Kp = 15                         # Proportional gain 20
Kd = 1                          # Derivative gain : 2
Tp = 200                        # Base speed of robot

# Initialize PID parameters
integral = 0
lastError = 0
derivative = 0
# Initialize forloop values
gsVal = 0
seconds = 0
measureChange = gs.value() - gs_offset
measureChangeTimer = 0
measureChangeTimer2 = 0
#gsMotor.position = 0
mLeft.position = 0
mRight.position = 0
lowestVal = 2500
lPrevVal = 0
rPrevVal = 0
endOfTrack = True
measuretime = False

#init gripper
gripper.run_forever()
gripper.position = 0
state = 'downstairs'
counter = 0
upstairsCounter = 0
hillAssist = True
gripper.run_to_abs_pos(position_sp=1750, speed_sp=1000, stop_action="hold")
# -------------------- PROGRAM FOLLOW LINE --------------------
while True:
    # Set up signal handler
    signal.signal(signal.SIGINT, signal_handler)
    prev_gsVal = gsVal
    gsVal = gs.value() - gs_offset
    
    # PID
    errorLeft = cs_left.value() - offsetLeft
    errorRight = cs_right.value() - offsetRight
    error = (errorLeft - errorRight) / 2.0
    derivative = error - lastError
    lastError = error

    Turn = Kp * error + Kd * derivative
    left_speed = Tp - Turn
    right_speed = Tp + Turn
        
    if (state != 'search' and state != 'turn_align' and state != 'findcan' and state != 'pickup' 
        and state != 'test_if_end_of_line'):
        left_speed, right_speed = saturateSpeed(left_speed, right_speed)
        mLeft.run_forever(speed_sp=left_speed)
        mRight.run_forever(speed_sp=right_speed)

    # Use gripper as support when climbing
    if hillAssist:
        if (time.time()-measureChangeTimer) > 1:
            print('Measured change: ', measureChange - gsVal)
            if (measureChange - gsVal) < -10: # Start hill climb
                Kp = 5
            elif (measureChange - gsVal) > 12: # end hill climb
                Kp = 15
                counter+=1
            else:
                print('Doing nothing')
            measureChange = gsVal
            measureChangeTimer = time.time()
            if counter == 2:
                counter = 0
                state = 'upstairs'
                hillAssist = False
                mLeft.position = 0
                mRight.position = 0

                
    # What to do when at the top level
    if state == 'upstairs':
        # Search for end of line          
        if (time.time()-measureChangeTimer) > 1:
            if ((abs(cs_left.value() - lPrevVal) < 3) and (abs(cs_right.value() - rPrevVal) < 3)):
                mLeft.stop(stop_action="hold")
                mRight.stop(stop_action="hold")                
                mLeft.position = 0
                mRight.position = 0
                state = 'test_if_end_of_line'
            lPrevVal = cs_left.value()
            rPrevVal = cs_right.value()
            measureChangeTimer = time.time()
        
        
    
    if state == 'test_if_end_of_line':
        if (cs_left.value() < 10) or (cs_right.value() < 10):
            endOfTrack = False

        if not endOfTrack:
            upstairsCounter = 0
            endOfTrack = True
            state = 'upstairs'

        if (upstairsCounter == 0):
            mLeft.run_to_abs_pos(position_sp=-200, speed_sp=200, stop_action="hold")
            mRight.run_to_abs_pos(position_sp=200, speed_sp=200, stop_action="hold")
            if mLeft.position < -95 and mRight.position > 95:
                upstairsCounter = 1
        elif (upstairsCounter == 1):
            mLeft.run_to_abs_pos(position_sp=200, speed_sp=200)
            mRight.run_to_abs_pos(position_sp=-200, speed_sp=200)
            if mLeft.position > 195 and mRight.position < -195:
                upstairsCounter = 2
        elif (upstairsCounter == 2):
            mLeft.run_to_abs_pos(position_sp=0, speed_sp=200)
            mRight.run_to_abs_pos(position_sp=0, speed_sp=200)
            if mLeft.position < 5 and mRight.position > -5:
                upstairsCounter = 3
        elif (upstairsCounter == 3):
            mLeft.run_to_abs_pos(position_sp=-200, speed_sp=200)
            mRight.run_to_abs_pos(position_sp=-200, speed_sp=200)
            if mLeft.position < -95 and mRight.position < -95:
                upstairsCounter = 4
                mLeft.position = 0
                mRight.position = 0
        elif (upstairsCounter == 4):
            mLeft.run_to_abs_pos(position_sp=-200, speed_sp=200)
            mRight.run_to_abs_pos(position_sp=200, speed_sp=200)
            if mLeft.position < -95 and mRight.position > 95:
                upstairsCounter = 5
        elif (upstairsCounter == 5):
            mLeft.run_to_abs_pos(position_sp=200, speed_sp=200)
            mRight.run_to_abs_pos(position_sp=-200, speed_sp=200)
            if mLeft.position > 195 and mRight.position < -195:
                state = 'turn_align'
                mLeft.stop(stop_action="coast")
                mRight.stop(stop_action="coast")
    

    if state == 'turn_align':
        mLeft.run_to_abs_pos(position_sp=-700, speed_sp=200, stop_action="hold") # Rotating left
        mRight.run_to_abs_pos(position_sp=700, speed_sp=200, stop_action="hold")
        gripper.run_to_abs_pos(position_sp=0, speed_sp=1000, stop_action="hold")
        if mLeft.position < (mLeft.position_sp + 5 ) and mRight.position > (mRight.position_sp - 5) :
            state = 'search'
    
    # Search for can
    if state == 'search':
        # After turning around, we keep turning to detect the can, and saves the position where we find the can.
        mLeft.run_to_abs_pos(position_sp=-1300, speed_sp=200, stop_action="hold") # Rotating left
        mRight.run_to_abs_pos(position_sp=1300, speed_sp=200, stop_action="hold")
        echoSensVal = echoSens.value()
        
        if (echoSensVal < lowestVal):
            lowestVal = echoSensVal
            savemLeftPos = mLeft.position
            savemRightPos = mRight.position

        if mLeft.position < (mLeft.position_sp + 5 ) and mRight.position > (mRight.position_sp - 5) :
            state = "findcan"
            mLeft.stop(stop_action="hold")
            mRight.stop(stop_action="hold")

        
    if state == "findcan":
        # find shortest distance
        mLeft.run_to_abs_pos(position_sp=savemLeftPos, speed_sp=200, stop_action="hold") # Rotating right
        mRight.run_to_abs_pos(position_sp=savemRightPos, speed_sp=200, stop_action="hold")
        if mLeft.position > (savemLeftPos - 5) and mRight.position < (savemRightPos + 5):
            state = 'pickup'
            movedist = lowestVal * 1.7
            mLeft.run_forever(speed_sp=-Tp)
            mRight.run_forever(speed_sp=-Tp)
            savemLeftPos = mLeft.position

    if state == 'pickup':
        echoSensVal = echoSens.value()
        print('[mLeft.pos, mRight.pos, echo] = [', mLeft.position, ', ', mRight.position, ', ' ,echoSensVal, ']')
        if mLeft.position < savemLeftPos-movedist and not measuretime:
            gripper.run_to_abs_pos(position_sp=1750, speed_sp=1000, stop_action="hold")
            measuretime = True
            measureChangeTimer = time.time()
            
        if measuretime == True and (time.time() - measureChangeTimer) > 0.3:   
            print('time: ', time.time() - measureChangeTimer)
            mLeft.stop(stop_action="hold")
            mRight.stop(stop_action="hold")
            if (time.time() - measureChangeTimer) > 2.0:
                state = 'downstairs'
                
    
    if state == "returnToLine":
        Kp = 20
        state = 'downstairs'
        


        

        



        
