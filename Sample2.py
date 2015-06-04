################################################################################
# Copyright (C) 2012-2013 Leap Motion, Inc. All rights reserved.               #
# Leap Motion proprietary and confidential. Not for distribution.              #
# Use subject to the terms of the Leap Motion SDK Agreement available at       #
# https://developer.leapmotion.com/sdk_agreement, or another agreement         #
# between Leap Motion and you, your company or other organization.             #
################################################################################

import Leap, sys, thread, time
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture

import roslib; roslib.load_manifest('irobot_mudd')
import rospy
import cv
import sensor_msgs.msg as sm
from std_msgs.msg import String
import time
import math


class Data: pass    # empty class for a generic data holder
D = Data()  # an object to hold our system's services and state



class SampleListener(Leap.Listener):
    finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
    bone_names = ['Metacarpal', 'Proximal', 'Intermediate', 'Distal']
    state_names = ['STATE_INVALID', 'STATE_START', 'STATE_UPDATE', 'STATE_END']

    global D

    D.last_time = 0
    D.current_time = 0
    D.delta_t = 0.5
    D.speed_scale = 0
    D.stop = 0
    D.pubL = 0
    D.pubR = 0
    D.reverse = 1

    D.x = 0
    D.z = 0
    D.slope = 0
    D.ang = 0
    D.timer = 0

    def on_init(self, controller):
        print "Initialized"
        self.pub = rospy.Publisher('text_data',String)
        rospy.init_node('sample')

    def on_connect(self, controller):
        print "Connected"

        # Enable gestures
        controller.enable_gesture(Leap.Gesture.TYPE_CIRCLE);
        controller.enable_gesture(Leap.Gesture.TYPE_KEY_TAP);
        controller.enable_gesture(Leap.Gesture.TYPE_SCREEN_TAP);
        controller.enable_gesture(Leap.Gesture.TYPE_SWIPE);

    def on_disconnect(self, controller):
        # Note: not dispatched when running in a debugger.
        print "Disconnected"

    def on_exit(self, controller):
        print "Exited"

    def on_frame(self, controller):

        # Get the most recent frame and report some basic information
        frame = controller.frame()

        # print "Frame id: %d, timestamp: %d, hands: %d, fingers: %d, tools: %d, gestures: %d" % (
        #       frame.id, frame.timestamp, len(frame.hands), len(frame.fingers), len(frame.tools), len(frame.gestures()))

        # Get hands
        for hand in frame.hands:

            """

            handType = "Left hand" if hand.is_left else "Right hand"

            print "  %s, id %d, position: %s" % (
                handType, hand.id, hand.palm_position)

            # Get the hand's normal vector and direction
            normal = hand.palm_normal
            direction = hand.direction

            # Calculate the hand's pitch, roll, and yaw angles
            print "  pitch: %f degrees, roll: %f degrees, yaw: %f degrees" % (
                direction.pitch * Leap.RAD_TO_DEG,
                normal.roll * Leap.RAD_TO_DEG,
                direction.yaw * Leap.RAD_TO_DEG)

            # Get arm bone
            arm = hand.arm
            print "  Arm direction: %s, wrist position: %s, elbow position: %s" % (
                arm.direction,
                arm.wrist_position,
                arm.elbow_position)
            """
            position = hand.palm_position
            D.x = hand.palm_position.x
            D.y = hand.palm_position.y % 255
            D.z = hand.palm_position.z
            #print position
                #print time
            
                            #print D.z
            D.ang =(math.degrees( math.atan2(D.x, D.z)))
            if(D.ang < 0):
                D.ang = D.ang +360
                    
            print D.timer
            D.slope = math.sqrt((D.x**2.0)+(D.z**2.0))
            if(D.timer > 50):      #print D.y  
                print D.ang     
                D.pubL = "data(" + str(D.ang) + "," + str(D.slope) + "," + str(D.y) + ")"
                self.pub.publish(String(D.pubL)) 
                D.timer = 0
                            

            last_command = ''
            D.timer += 1
            #print D.timer
            # Get fingers
            #for finger in hand.fingers:

                #if self.finger_names[finger.type()] == 'Index':
                #     print "    %s finger, id: %d, length: %fmm, width: %fmm" % (
                #         self.finger_names[finger.type()],
                #         finger.id,
                #         finger.length,
                #         finger.width)

                # Get bones
                    #for b in range(0, 4):
                        #bone = finger.bone(b)
                        #if self.bone_names[bone.type] == 'Distal':
                        # print "      Bone: %s, start: %s, end: %s, direction: %s" % (
                            # print "Bone: %s, Direction:  %s" % (
                            #     self.bone_names[bone.type],
                            #     # bone.prev_joint,
                            #     # bone.next_joint,
                            #     bone.direction)
                            #bone.direction[0] is x
                            #bone.direction[1] is z
                            #bone.direction[2] is y






                            #D.slope = D.x/D.y




                            # D.speed_scale = abs(int((bone.direction.y)*300))
                            # if(D.stop):
                            #     if bone.direction.x > 0:
                            #         left_wheel = D.reverse*int((D.speed_scale*3.0/4.0))
                            #         right_wheel = D.reverse*int(D.speed_scale)
                            #         D.pubL = "D.tank(" + str(left_wheel) + "," + str(right_wheel) + ")"
                            #         self.pub.publish(String(D.pubL))
                            #         # last_command = 'left'
                            #     if bone.direction.x <= 0:
                            #         left_wheel = D.reverse*int(D.speed_scale)
                            #         right_wheel = D.reverse*int((D.speed_scale*3.0/4.0))
                            #         D.pubR = "D.tank(" + str(left_wheel) + "," + str(right_wheel) + ")"
                            #         self.pub.publish(String(D.pubR))  

                            # else:
                            #     self.pub.publish("D.tank(0,0)")
                              
                                # last_command = 'right'

                            # if bone.direction.y > 0:

                            # if bone.direction.y <= 0:

                            # if bone.direction[2] > 0:

                            # if bone.direction[2] <= 0:
                            # else:
                            #     self.pub.publish(String('stop'))
                            #     last_command = 'stop'

        
        # D.current_time = time.time()
        # if D.current_time > D.last_time + D.delta_t:
        #         # print "I would publish", last_command
        #     D.last_time = D.current_time

        # # Get tools
        # for tool in frame.tools:
        #     print "  Tool id: %d, position: %s, direction: %s" % (
        #         tool.id, tool.tip_position, tool.direction)

        # # Get gestures
        # for gesture in frame.gestures():
        #     if gesture.type == Leap.Gesture.TYPE_CIRCLE:
        #         circle = CircleGesture(gesture)

        #         # Determine clock direction using the angle between the pointable and the circle normal
        #         if circle.pointable.direction.angle_to(circle.normal) <= Leap.PI/2:
        #             clockwiseness = "clockwise"
        #             if D.stop == 1:
        #                 D.stop = 0

        #             else:
        #                 D.stop = 1
        #         else:
        #             clockwiseness = "counterclockwise"
        #             if D.reverse == -1:
        #                 D.reverse = 1

        #             else: 
        #                 D.reverse = -1

                # Calculate the angle swept since the last frame
                #swept_angle = 0
                # if circle.state != Leap.Gesture.STATE_START:
                #     previous_update = CircleGesture(controller.frame(1).gesture(circle.id))
                #     swept_angle =  (circle.progress - previous_update.progress) * 2 * Leap.PI

                # print "  Circle id: %d, %s, progress: %f, radius: %f, angle: %f degrees, %s" % (
                #         gesture.id, self.state_names[gesture.state],
                #         circle.progress, circle.radius, swept_angle * Leap.RAD_TO_DEG, clockwiseness)

            # if gesture.type == Leap.Gesture.TYPE_SWIPE:
            #     swipe = SwipeGesture(gesture)
            #     print "  Swipe id: %d, state: %s, position: %s, direction: %s, speed: %f" % (
            #             gesture.id, self.state_names[gesture.state],
            #             swipe.position, swipe.direction, swipe.speed)
            #     #
            #     #
            #     #
            #     self.pub.publish(String('stop'))

            # if gesture.type == Leap.Gesture.TYPE_KEY_TAP:
            #     keytap = KeyTapGesture(gesture)
            #     print "  Key Tap id: %d, %s, position: %s, direction: %s" % (
            #             gesture.id, self.state_names[gesture.state],
            #             keytap.position, keytap.direction )

            # if gesture.type == Leap.Gesture.TYPE_SCREEN_TAP:
            #     screentap = ScreenTapGesture(gesture)
            #     print "  Screen Tap id: %d, %s, position: %s, direction: %s" % (
            #             gesture.id, self.state_names[gesture.state],
            #             screentap.position, screentap.direction )

        # if not (frame.hands.is_empty and frame.gestures().is_empty):
        #     print ""

    # def state_string(self, state):
    #     if state == Leap.Gesture.STATE_START:
    #         return "STATE_START"

    #     if state == Leap.Gesture.STATE_UPDATE:
    #         return "STATE_UPDATE"

    #     if state == Leap.Gesture.STATE_STOP:
    #         return "STATE_STOP"

    #     if state == Leap.Gesture.STATE_INVALID:
    #         return "STATE_INVALID"

def main():
    # Create a sample listener and controller
    listener = SampleListener()
    controller = Leap.Controller()

    # Have the sample listener receive events from the controller
    controller.add_listener(listener)

    # Keep this process running until Enter is pressed
    print "Press Enter to quit..."
    try:
        sys.stdin.readline()
    except KeyboardInterrupt:
        pass
    finally:
        # Remove the sample listener when done
        controller.remove_listener(listener)


if __name__ == "__main__":
    main()
