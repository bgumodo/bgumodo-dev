#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from diagnostic_msgs.msg import KeyValue
from rosplan_dispatch_msgs.msg import ActionDispatch
from rosplan_dispatch_msgs.msg import ActionFeedback
from rosplan_knowledge_msgs.msg import KnowledgeItem
import mongodb_store.message_store

from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceRequest
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceResponse
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService

from rosplan_knowledge_msgs.srv import GetAttributeServiceRequest
from rosplan_knowledge_msgs.srv import GetAttributeServiceResponse
from rosplan_knowledge_msgs.srv import GetAttributeService

from komodo_speech.msg import KomodoSpeechSayCommand
from komodo_speech.msg import KomodoSpeechRecCommand
from komodo_speech.msg import KomodoSpeechRecResult

from std_msgs.msg import Header

class Handler(object):

    def __init__(self):
    	rospy.loginfo("Waiting for actions")

        #self.move_pub = rospy.Publisher("navigation/move_location_res", String, queue_size=10)
        self.btn_loc_pub = rospy.Publisher("vision/observe_btn_res", PoseStamped, queue_size=10)
        self.btn_press_pub = rospy.Publisher("arm/press_door_button_res", String, queue_size=10)
        #self.move_through_door_pub = rospy.Publisher("navigation/move_through_door_res", String, queue_size=10)
        self.order_coke_pub = rospy.Publisher("speech/order_coke_res", String, queue_size=10)
        self.can_loc_pub = rospy.Publisher("vision/observe_can_res", PoseStamped, queue_size=10)
        self.can_grab_pub = rospy.Publisher("arm/grab_can_res", String, queue_size=10)
        self.request_elevator_pub = rospy.Publisher("speech/request_open_door_res", String, queue_size=10)
        self.serve_can_pub = rospy.Publisher("arm/serve_can_res", String, queue_size=10)

    	self.speech_rec_pub = rospy.Publisher("/KomodoSpeech/rec_command", KomodoSpeechRecCommand, queue_size=10)
    	self.speech_ask_pub = rospy.Publisher("/KomodoSpeech/say_command", KomodoSpeechSayCommand, queue_size=10)

        #rospy.Subscriber("navigation/move_location_cmd", String, self.move_callback)
        rospy.Subscriber("vision/observe_btn_cmd", String, self.observe_btn_callback)
        rospy.Subscriber("arm/press_door_button_cmd", PoseStamped, self.press_btn_callback)
        #rospy.Subscriber("navigation/move_through_door_cmd", String, self.move_through_door_callback)
        rospy.Subscriber("speech/order_coke_cmd", String, self.order_coke_callback)
        rospy.Subscriber("vision/observe_can_cmd", String, self.observe_can_callback)
        rospy.Subscriber("arm/grab_can_cmd", PoseStamped, self.grab_can_callback)
        rospy.Subscriber("speech/request_open_door_cmd", String, self.request_open_door_callback)
        rospy.Subscriber("arm/serve_can_cmd", String, self.serve_can_callback)

        rospy.spin()

    def move_callback(self, data):
    	rospy.loginfo("Dispatching action (move): Moving the robot to location: %s", data.data)
    	rospy.sleep(3.)
    	self.move_pub.publish("Success")

    def observe_btn_callback(self, data):
    	rospy.loginfo("Dispatching action (observe button location): Observing elevator button location")
    	rospy.sleep(3.)
    	pos = PoseStamped()
    	pos.header.frame_id = "1"
    	self.btn_loc_pub.publish(pos)
    
    def press_btn_callback(self, data):
    	rospy.loginfo("Dispatching action (press button): Successfuly located elevator button. Pressing button..")
        rospy.loginfo("Button at: %s", data)
    	rospy.sleep(3.)
    	self.btn_press_pub.publish("Success")
    
    def move_through_door_callback(self, data):
    	rospy.loginfo("Dispatching action (move through door): moving through door to location: " + data.data)
    	# command = KomodoSpeechSayCommand()
    	# command.header = Header()
    	# command.text_to_say = "Please choose " + data.data + " for me"
    	# rospy.sleep(1.)
    	# self.speech_ask_pub.publish(command)
    	# rospy.sleep(5.)
    	# command = KomodoSpeechRecCommand()
    	# command.header = Header()
    	# command.cmd = 'start'
    	# command.cat = 'floor'
    	# self.speech_rec_pub.publish(command)
        rospy.sleep(3.)
    	#result = rospy.wait_for_message("/KomodoSpeech/rec_result", KomodoSpeechRecResult, timeout=120)
    	self.move_through_door_pub.publish("Success")
    
    def order_coke_callback(self, data):
    	rospy.loginfo("Dispatching action (order coke): Ordering coke")
    	command = KomodoSpeechSayCommand()
    	command.header = Header()
    	command.text_to_say = "Can I have a can of coke please"
    	rospy.sleep(1.)
    	self.speech_ask_pub.publish(command)
    	rospy.sleep(6.)
    	command = KomodoSpeechRecCommand()
    	command.header = Header()
    	command.cmd = 'start'
    	command.cat = 'coke'
    	self.speech_rec_pub.publish(command)

    	#result = rospy.wait_for_message("/KomodoSpeech/rec_result", KomodoSpeechRecResult, timeout=120)
    	
    	self.order_coke_pub.publish("Success")
    
    def observe_can_callback(self, data):
    	rospy.loginfo("Dispatching action (observe coke can location): Got positive feedback. Observing can location..")
    	rospy.sleep(3.)
    	pos = PoseStamped()
    	pos.header.frame_id = "1"
    	self.can_loc_pub.publish(pos)
    
    def grab_can_callback(self, data):
    	rospy.loginfo("Successfuly located can. Grabbing can..")
    	rospy.sleep(3.)
    	self.can_grab_pub.publish("Success")
    
    def request_open_door_callback(self, data):
    	rospy.loginfo("Dispatching action (request open door): Asking for door open (gripper not free)")
    	command = KomodoSpeechSayCommand()
    	command.header = Header()
    	command.text_to_say = "Can you open the door for me please"
    	rospy.sleep(1.)
    	self.speech_ask_pub.publish(command)
    	rospy.sleep(6.)
    	command = KomodoSpeechRecCommand()
    	command.header = Header()
    	command.cmd = 'start'
    	command.cat = 'yes_no'
    	self.speech_rec_pub.publish(command)

    	#result = rospy.wait_for_message("/KomodoSpeech/rec_result", KomodoSpeechRecResult, timeout=120)
    	
    	self.request_elevator_pub.publish("Success")
    
    def serve_can_callback(self, data):
    	rospy.loginfo("Dispatching action (serve can): Serving the can of coke")
    	command = KomodoSpeechSayCommand()
    	command.header = Header()
    	command.text_to_say = "Here you go"
    	rospy.sleep(1.)
    	self.speech_ask_pub.publish(command)
    	self.serve_can_pub.publish("Success")


if __name__ == '__main__':
	rospy.init_node('dummy_action_listener', anonymous=False)

	Handler()
