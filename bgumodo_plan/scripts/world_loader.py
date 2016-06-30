#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Header
from diagnostic_msgs.msg import KeyValue
from rosplan_dispatch_msgs.msg import ActionDispatch
from rosplan_dispatch_msgs.msg import ActionFeedback
from rosplan_knowledge_msgs.msg import KnowledgeItem

from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceRequest
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceResponse
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService

from rosplan_knowledge_msgs.srv import GetAttributeServiceRequest
from rosplan_knowledge_msgs.srv import GetAttributeServiceResponse
from rosplan_knowledge_msgs.srv import GetAttributeService
import mongodb_store.message_store

from komodo_speech.msg import KomodoSpeechRecCommand
from komodo_speech.msg import KomodoSpeechRecResult


def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "Dispatched: %s", data.name)
    
def listener():
	rospy.loginfo("Waiting for action dispatch")

	rospy.Subscriber("kcl_rosplan/action_dispatch", ActionDispatch, callback)

	rospy.spin()

def createInstances():
	update_knowledge_client = rospy.ServiceProxy("/kcl_rosplan/update_knowledge_base", KnowledgeUpdateService)
	add_knowledge_type = KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE

	knowledge = KnowledgeItem()
	knowledge.knowledge_type = KnowledgeItem.INSTANCE
	knowledge.instance_type = "obj"
	instance_list = ["door1","zone1","door_zone_1","door1_button","room_2","zone2","door_zone_2","room_4"]
	#"E1","zone1","E1_entrance_f1","E1_keypad_f1","room_2","room_4", \
	#"coke_stand","zone2","E1_entrance_f2","E1_keypad_f2","room3","room4"]

	for instance in instance_list:
		knowledge.instance_name = instance
		update_response = update_knowledge_client(add_knowledge_type, knowledge)
		if (update_response.success is not True):
			rospy.loginfo("Failed updating KMS with instance: %s", knowledge.instance_name)
		else:
			rospy.loginfo("Updated KMS with instance: %s", knowledge.instance_name)

	#message_store = mongodb_store.message_store.MessageStoreProxy()
	#message_store.update_named("room_2",String("room one"), upsert=True)
	#message_store.update_named("room_4",String("room two"), upsert=True)

def createFacts():
	update_knowledge_client = rospy.ServiceProxy("/kcl_rosplan/update_knowledge_base", KnowledgeUpdateService)
	addFact("arm_free", [] , update_knowledge_client)
	addFact("robot_at", [["loc","room_2"]] , update_knowledge_client)
	addFact("robot_at_zone", [["zone","zone1"]] , update_knowledge_client)
	addFact("loc_in_zone", [["loc","door_zone_1"], ["zone","zone1"]] , update_knowledge_client)
	addFact("loc_in_zone", [["loc","room_2"], ["zone","zone1"]] , update_knowledge_client)
	addFact("loc_in_zone", [["loc","room_4"], ["zone","zone2"]] , update_knowledge_client)
	addFact("loc_in_zone", [["loc","door_zone_2"], ["zone","zone2"]] , update_knowledge_client)
	addFact("has_door_button", [["loc","door_zone_1"], ["door","door1"]] , update_knowledge_client)
	addFact("has_door_button", [["loc","door_zone_2"], ["door","door1"]] , update_knowledge_client)
	addFact("door_entrance", [["door","door1"], ["zone","zone1"], ["loc","door_zone_1"]] , update_knowledge_client)
	addFact("door_entrance", [["door","door1"], ["zone","zone2"], ["loc","door_zone_2"]] , update_knowledge_client)
	addFact("has_coke_stand", [["loc","room_4"]] , update_knowledge_client)
	
def addFact(name, params, client):
	add_knowledge_type = KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE
	
	knowledge = KnowledgeItem()
	knowledge.knowledge_type = KnowledgeItem.FACT
	knowledge.attribute_name = name
	for param in params:
		pair = KeyValue()
		pair.key = param[0]
		pair.value = param[1]
		knowledge.values.append(pair)
	update_response = client(add_knowledge_type, knowledge)
	if (update_response.success is not True):
		rospy.loginfo("Failed updating KMS with attribute: %s", knowledge.attribute_name)
	else:
		rospy.loginfo("Updated KMS with attribute: %s", knowledge.attribute_name)	

def createGoal():
	rosplan_pub = rospy.Publisher("/kcl_rosplan/planning_commands", String, queue_size=10)
	speech_pub = rospy.Publisher("/KomodoSpeech/rec_command", KomodoSpeechRecCommand, queue_size=10)
	command = KomodoSpeechRecCommand()
	command.header = Header()
	command.cmd = 'start'
	command.cat = 'cmd'
	rospy.sleep(2.)
	speech_pub.publish(command)

	result = rospy.wait_for_message("/KomodoSpeech/rec_result", KomodoSpeechRecResult, timeout=30)

	if result and result.cat == "cmd" and result.success is True and result.phrase_id == 8:
		attribute_query_client = rospy.ServiceProxy("/kcl_rosplan/get_current_knowledge", GetAttributeService)
		knowledge_items = attribute_query_client.call("robot_at")
		if len(knowledge_items.attributes) > 1:
			rospy.loginfo("Failed to add goal. Robot in two places (robot_at)")
			return
		current_location = knowledge_items.attributes[0].values[0].value


		update_knowledge_client = rospy.ServiceProxy("/kcl_rosplan/update_knowledge_base", KnowledgeUpdateService)
		add_knowledge_type = KnowledgeUpdateServiceRequest.ADD_GOAL

		knowledge = KnowledgeItem()
		knowledge.knowledge_type = KnowledgeItem.FACT
		knowledge.attribute_name = "coke_at"
		pair = KeyValue()
		pair.key = "loc" 
		pair.value = current_location #room_2
		knowledge.values.append(pair)
		update_response = update_knowledge_client(add_knowledge_type, knowledge)
		if (update_response.success is not True):
			rospy.loginfo("Failed to add goal of attribute: %s", knowledge.attribute_name)
		else:
			rospy.loginfo("Added goal of attribute: %s", knowledge.attribute_name)

			rosplan_pub.publish(String("plan"))
	else:
		rospy.loginfo("Couldn't get command. Closing")

if __name__ == '__main__':
	rospy.init_node('world_loader', anonymous=False)

	createInstances()
	createFacts()
	createGoal()


	listener()
