#!/usr/bin/env python
import os
import re
import sys
import rospy
import time
from kortex_driver.srv import *
from kortex_driver.msg import *

def commandobj(object_description, command_id):
    #Defines Directory Path
    path = os.path.dirname(os.path.realpath(__file__))
    path = path.replace("/scripts", "")
    path = path + "/ycb_gazebo_sdf/"
    #Defines specific command arguments for the Object
    object_arguments = object_description.split(" ", 1)
    #Defines type of command: 1-Spawn, 0-Despawn
    if(command_id == 1):
        return "gz model -f " + path + object_arguments[0] + "/model.sdf -m " + object_arguments[0] + " " + object_arguments[1]
    elif(command_id == 0):
        return "gz model -m " + object_arguments[0] + " -d"
    else:
        raise NameError("Command not found!")
        return "error"

def argument_identifier():
    path = os.path.dirname(os.path.realpath(__file__))
    path = path + '/objectslist.txt'
    try:
        objfile = open(path,'r')
        load_data = objfile.readlines()
    finally:
        objfile.close()

    objlist = ["empty"]
    for i in range(len(sys.argv)):
        for j in range(len(load_data)):
            token = "^" + sys.argv[i]
            check = re.search(token, load_data[j])
            if check:
                if(objlist[0] == "empty"):
                    objlist[0] = load_data[j][:-1]  
                    break       
                else:
                    objlist.append(load_data[j][:-1])
                    break
    return objlist

class ExampleCartesianActionsWithNotifications:
    def __init__(self):
        try:
            rospy.init_node('example_cartesian_poses_with_notifications_python')

            self.HOME_ACTION_IDENTIFIER = 2

            self.action_topic_sub = None
            self.all_notifs_succeeded = True

            # Get node params
            self.robot_name = rospy.get_param('~robot_name', "my_gen3")

            rospy.loginfo("Using robot_name " + self.robot_name)

            # Init the action topic subscriber
            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            self.last_action_notif_type = None

            # Init the services
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
            rospy.wait_for_service(set_cartesian_reference_frame_full_name)
            self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)
        except:
            self.is_init_success = False
        else:
            self.is_init_success = True

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                self.all_notifs_succeeded = False
                return False
            else:
                time.sleep(0.01)

    def example_clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True

    def example_home_the_robot(self):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        self.last_action_notif_type = None
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                return self.wait_for_action_end_or_abort()

    def example_set_cartesian_reference_frame(self):
        # Prepare the request with the frame we want to set
        req = SetCartesianReferenceFrameRequest()
        req.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED

        # Call the service
        try:
            self.set_cartesian_reference_frame()
        except rospy.ServiceException:
            rospy.logerr("Failed to call SetCartesianReferenceFrame")
            return False
        else:
            rospy.loginfo("Set the cartesian reference frame successfully")
            return True

        # Wait a bit
        rospy.sleep(0.25)

    def example_subscribe_to_a_robot_notification(self):
        # Activate the publishing of the ActionNotification
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")

        rospy.sleep(1.0)

        return True

    def main(self):
        # For testing purposes
        success = self.is_init_success
        try:
            rospy.delete_param("/kortex_examples_test_results/cartesian_poses_with_notifications_python")
        except:
            pass

        if success:

            #*******************************************************************************
            # Make sure to clear the robot's faults else it won't move if it's already in fault
            success &= self.example_clear_faults()
            #*******************************************************************************

            #*******************************************************************************
            # Subscribe to ActionNotification's from the robot to know when a cartesian pose is finished
            success &= self.example_subscribe_to_a_robot_notification()
            #*******************************************************************************

            #*******************************************************************************
            # Start the example from the Home position
            success &= self.example_home_the_robot()
            #*******************************************************************************

            #*******************************************************************************
            # Set the reference frame to "Mixed"
            success &= self.example_set_cartesian_reference_frame()

            # Define Objects to be used during the Test
            if(len(sys.argv) >= 2):
                list_of_objects = argument_identifier()
                if list_of_objects[0] == "empty":
                    objectslist = ["006_mustard_bottle_textured -x 0.6 -y 0.02675 -z 0 -R 0 -P 0 -Y 0.4"] #default
                else:
                    objectslist = list_of_objects
            else:
                objectslist = ["006_mustard_bottle_textured -x 0.6 -y 0.02675 -z 0 -R 0 -P 0 -Y 0.4"] #default

            for val in range(len(objectslist)):

                #*******************************************************************************
                # Spawn Object
                os.system(commandobj(objectslist[val], 1))

                # Prepare and send pose 1
                my_cartesian_speed = CartesianSpeed()
                my_cartesian_speed.translation = 0.1 # m/s
                my_cartesian_speed.orientation = 15  # deg/s

                my_constrained_pose = ConstrainedPose()
                my_constrained_pose.constraint.oneof_type.speed.append(my_cartesian_speed)

                my_constrained_pose.target_pose.x = 0.5
                my_constrained_pose.target_pose.y = 0
                my_constrained_pose.target_pose.z = 0.10
                my_constrained_pose.target_pose.theta_x = 90
                my_constrained_pose.target_pose.theta_y = 0
                my_constrained_pose.target_pose.theta_z = 90

                req = ExecuteActionRequest()
                req.input.oneof_action_parameters.reach_pose.append(my_constrained_pose)
                req.input.name = "pose1"
                req.input.handle.action_type = ActionType.REACH_POSE
                req.input.handle.identifier = 1001

                rospy.loginfo("Sending pose 1...")
                self.last_action_notif_type = None
                try:
                    self.execute_action(req)
                except rospy.ServiceException:
                    rospy.logerr("Failed to send pose 1")
                    success = False
                else:
                    rospy.loginfo("Waiting for pose 1 to finish...")

                self.wait_for_action_end_or_abort()

                # Prepare and send pose 2
                req.input.handle.identifier = 1002
                req.input.name = "pose2"

                my_constrained_pose.target_pose.x = 0.7

                req.input.oneof_action_parameters.reach_pose[0] = my_constrained_pose

                rospy.loginfo("Sending pose 2...")
                self.last_action_notif_type = None
                try:
                    self.execute_action(req)
                except rospy.ServiceException:
                    rospy.logerr("Failed to send pose 2")
                    success = False
                else:
                    rospy.loginfo("Waiting for pose 2 to finish...")

                self.wait_for_action_end_or_abort() 

                # Prepare and send pose 3
                req.input.handle.identifier = 1003
                req.input.name = "pose3"

                my_constrained_pose.target_pose.x = 0.45
                my_constrained_pose.target_pose.z = 0.3

                req.input.oneof_action_parameters.reach_pose[0] = my_constrained_pose

                rospy.loginfo("Sending pose 3...")
                self.last_action_notif_type = None
                try:
                    self.execute_action(req)
                except rospy.ServiceException:
                    rospy.logerr("Failed to send pose 3")
                    success = False
                else:
                    rospy.loginfo("Waiting for pose 3 to finish...")

                self.wait_for_action_end_or_abort()

                success &= self.all_notifs_succeeded

                # Despawn Object
                os.system(commandobj(objectslist[val], 0))

                success &= self.all_notifs_succeeded

            # Home Robot
            os.system("rosrun kortex_gazebo home_robot.py")

        # For testing purposes
        rospy.set_param("/kortex_examples_test_results/cartesian_poses_with_notifications_python", success)

        if not success:
            rospy.logerr("The example encountered an error.")

if __name__ == "__main__":
    ex = ExampleCartesianActionsWithNotifications()
    ex.main()
