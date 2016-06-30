#!/usr/bin/env python
# always
import rospy
import sys
from rosplan_dispatch_msgs.msg import ActionDispatch
from rosplan_dispatch_msgs.msg import ActionFeedback
from rosplan_knowledge_msgs.msg import KnowledgeItem
from diagnostic_msgs.msg import KeyValue

from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceRequest
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceResponse
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService

from rosplan_knowledge_msgs.srv import GetInstanceServiceRequest
from rosplan_knowledge_msgs.srv import GetInstanceServiceResponse
from rosplan_knowledge_msgs.srv import GetInstanceService

import mongodb_store.message_store
from std_msgs.msg import String

# From Glue
from geometry_msgs.msg import PoseStamped

class PLP_request_elevator_order_action_Parameters(object):
    def __init__(self):
        # self.callback = None
        # Execution Parameters
        # Input Parameters
        # Output Parameters
        self.result = None

    def set_result(self, a_result):
        self.result = a_result

class PLP_request_elevator_order_action_Variables(object):
    def __init__(self):
    	# Nothing in this action
        self.none = None


class Dispatcher(object):

    def __init__(self):
        # self.plp_constants = {
        #     "MIN_LOC_ERROR": 5,  # meters
        #     "BOBCAT_SIZE": (3.5, 2, 1.7)
        # }

        # Setup internal PLP objects.
        # self.plp = None
        self.update_knowledge_client = rospy.ServiceProxy("/kcl_rosplan/update_knowledge_base", KnowledgeUpdateService)
        self.message_store = mongodb_store.message_store.MessageStoreProxy()
        self.action_feedback_pub = rospy.Publisher("/kcl_rosplan/action_feedback", ActionFeedback, queue_size=10)

        self.action_publisher = rospy.Publisher("speech/order_elevator_cmd", String, queue_size=10)

        self.plp_params = PLP_request_elevator_order_action_Parameters()
        self.plp_vars = PLP_request_elevator_order_action_Variables()

        rospy.Subscriber("/kcl_rosplan/action_dispatch", ActionDispatch, self.dispatch_action)

        rospy.Subscriber("speech/order_elevator_res", String, self.result_updated)

    def result_updated(self, a_result):
        self.plp_params.set_result(a_result.data)
        # Because output param
        self.message_store.update_named("output_result",a_result)
        #
        self.parameters_updated()


    def parameters_updated(self):
        self.calculate_variables()
        if self.detect_success():
    		rospy.loginfo("request_elevator_order_action_dispatcher: detected success")
    		self.update_success()
    		self.reset_dispatcher()
        elif self.detect_failure():
    		rospy.loginfo("request_elevator_order_action_dispatcher: detected failure")
    		self.update_fail()
    		self.reset_dispatcher()

    def calculate_variables(self):
    	# Nothing in this action
    	self.plp_vars.none = None

    def detect_success(self):
    	return self.plp_params.result == "Success"
        #code to detect success over self.plp_parameters / self.plp_variables

    def detect_failure(self):
    	return self.plp_params.result == "Failed"
        #code to detect failure over self.plp_parameters / self.plp_variables

    def dispatch_action(self, action):
        if not action.name == "request_elevator_order":
            return
        self.current_action = action
        # TODO: delete (for test)
        # temp = KeyValue()
        # temp.key = "curr_loc"
        # temp.value = "room4"
        # self.current_action.parameters.append(temp)
        #
        # from glue pddl->plp parameter glue
        # for pair in action.parameters:
        #     if pair.key == "loc":
        #         self.target_location = self.message_store.query_named(pair.value, String._type, False)
        #         # if not saved special value in DB, use the name (check if list returned is empty)
        #         if not self.target_location:
        #             self.target_location = pair.value

        # get values from action outputs history
        # if not self.target_location:
        #     self.target_location = self.message_store.query_named("output_target_location", String._type, False)

        # dispatch action if got everything
        if self.check_can_dispatch():
            self.action_publisher.publish("Go")
        else:
        	rospy.loginfo("Failed at running action: %s. Conditions not met for dispatch", action.name)


    def check_can_dispatch(self):
        # if not (self.target_location is None):
        #     return True
        # return False
        return True

    def update_success(self):
        # GENERATED CODE
        parametersDic = self.toDictionary(self.current_action.parameters)
        self.changeKMSFact("ordered", [["elevator", parametersDic["elevator"]]], KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE)
        # END
        actionFeedback = ActionFeedback()
        actionFeedback.action_id = self.current_action.action_id
        actionFeedback.status = "action achieved"
        self.action_feedback_pub.publish(actionFeedback)

    def update_fail(self):
        actionFeedback = ActionFeedback()
        actionFeedback.action_id = self.current_action.action_id
        actionFeedback.status = "action failed"
        self.action_feedback_pub.publish(actionFeedback)

    def changeKMSFact(self, name, params, changeType):
		knowledge = KnowledgeItem()
		knowledge.knowledge_type = KnowledgeItem.FACT
		knowledge.attribute_name = name
		for param in params:
			pair = KeyValue()
			pair.key = param[0]
			pair.value = param[1]
			knowledge.values.append(pair)
		update_response = self.update_knowledge_client(changeType, knowledge)
		if (update_response.success is not True):
			rospy.loginfo("Failed updating KMS with attribute: %s", knowledge.attribute_name)
		else:
			rospy.loginfo("Updated KMS with attribute: %s", knowledge.attribute_name)

    def reset_dispatcher(self):
        # self.plp = None
        # self.plp_params.callback = None
        self.plp_params = PLP_request_elevator_order_action_Parameters()
        self.plp_vars = PLP_request_elevator_order_action_Variables()

    def toDictionary(self, pairs):
    	result = []
    	for pair in pairs:
    		result.append((pair.key,pair.value))
    	return dict(result)


if __name__ == '__main__':
    try:
        rospy.init_node("plp_request_elevator_order_action_dispatcher", anonymous=False)
        rospy.loginfo("Starting request_elevator_order action dispatcher. Waiting for command")
        Dispatcher()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
