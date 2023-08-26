
import typing
from abc import ABC, abstractmethod

import rclpy
from rclpy.time import Time
from rclpy.node import Node
from rclpy.callback_groups import CallbackGroup

from action_msgs.msg import GoalStatus

from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access

from behaviour_tree_rclpy.action.client import ActionClient, ResultMessage, ClientGoalHandle

ActionType = typing.TypeVar('ActionType')
ActionGoal = typing.TypeVar('ActionGoal')
ActionResult = typing.TypeVar('ActionResult')
ActionFeedback = typing.TypeVar('ActionFeedback')

to_sec = 1e-9

class BtActionNode(Behaviour, ABC):
    loop_safty_factor = 0.9
    def __init__(
        self, 
        name: str, 
        action_topic: str, 
        action_type: ActionType,
        send_goal_timeout: float,
        server_timeout: float,
    ):
        super().__init__(name)
        
        self.blackboard = self.attach_blackboard_client(name)
        self.blackboard.register_key('node', access=Access.READ, required=True)
        self.blackboard.register_key('bt_loop_duration', access=Access.READ, required=True)
        self.blackboard.register_key('action_cb_group', access=Access.READ, required=True)
        
        self.node: Node = self.blackboard.node
        self.action_topic: str = action_topic
        self.action_type: ActionType = action_type
        self.bt_loop_duration: float = self.blackboard.bt_loop_duration * self.loop_safty_factor
        self.send_goal_timeout:float = send_goal_timeout
        self.server_timeout: float = server_timeout
        self.cb_group: CallbackGroup = self.blackboard.action_cb_group
        
        self.goal: typing.Optional[ActionGoal] = None
        self.goal_handle: typing.Optional[ClientGoalHandle] = None
        self.result: typing.Optional[ResultMessage] = None
        self.sent_time: typing.Optional[Time] = None
        self.result_avaliable: bool = False
        self.action_feedback: typing.Optional[ActionFeedback] = None
        self.get_result_future: typing.Optional[rclpy.Future] = None
        self.send_goal_future: typing.Optional[rclpy.Future] = None
        
    def setup(self):
        self.action_client = ActionClient(
            self.node,
            self.action_type,
            self.action_topic,
            callback_group=self.cb_group,
        )

        self.node.get_logger().info(
            '[{}] Waiting for {} action server]'.format(self.name, self.action_topic))
        
        if not self.action_client.wait_for_server(1.0):
            self.node.get_logger().error(
                '[{}] {} action server not available after waiting for 1 s'.format(self.name, self.action_topic))

            raise RuntimeError('Action server {} not available'.format(self.action_topic))
        
        self.node.get_logger().info('[{}] BtActionNode initialized'.format(self.name))
        
    def initialise(self) -> None:
        self.goal = None
        self.goal_handle = None
        self.result = None
        self.sent_time = None
        self.result_avaliable = False
        self.action_feedback = None
        self.get_result_future = None
        self.send_goal_future = None
        self.status = Status.INVALID
    
    def update(self) -> Status:
        
        if self.status == Status.INVALID:
            should_sent_goal, self.goal = self.on_update()
            
            if not should_sent_goal:
                return Status.FAILURE
            self.send_new_goal()
            
        try:
            if self.send_goal_future:
                elapsed_time = (self.node.get_clock().now() - self.sent_time).nanoseconds*to_sec
                remaining_time = self.send_goal_timeout - elapsed_time
                timeout = remaining_time if self.bt_loop_duration > remaining_time else self.bt_loop_duration
                if not self.is_send_goal_future_complete(elapsed_time, timeout):
                    elapsed_time += timeout
                    if elapsed_time < self.send_goal_timeout:
                        return Status.RUNNING
                    
                    self.node.get_logger().debug(
                        '[{}] Timed out while waiting for {} action server to acknowledge goal request'.format(self.name, self.action_topic))
                    self.send_goal_future = None
                    self.feedback_message = "timeout while waiting for GoalResponse"
                    return Status.FAILURE
            
            elapsed_time = (self.node.get_clock().now() - self.sent_time).nanoseconds*to_sec
            remaining_time = self.server_timeout - elapsed_time
            timeout = remaining_time if self.bt_loop_duration > remaining_time else self.bt_loop_duration
                
            if rclpy.ok() and not self.get_result_future.done():
                self.on_wait_for_result(self.action_feedback)
                
                self.action_feedback = None
            
                self.node.executor.spin_once(timeout)

            if not self.result_avaliable:
                elapsed_time += timeout
                if elapsed_time < self.server_timeout:
                    return Status.RUNNING
                else:
                    
                    self.feedback_message = "timeout while waiting for result"
                    return Status.FAILURE
                
        except RuntimeError as e:
            self.node.get_logger().error('[{}] catch error : {} when tick behaviour'.format(self.name,e))
            if str(e) == 'Goal was rejected by the action server' or str(e) == 'Sending Goal Failed':
                return Status.FAILURE
            else:
                raise RuntimeError(e)
            
        goal_status = self.result.status
        result = self.result.result
        if goal_status == GoalStatus.STATUS_SUCCEEDED:
            status = self.on_success(result)
        elif goal_status == GoalStatus.STATUS_CANCELED:
            status = self.on_cancelled(result)
        elif goal_status == GoalStatus.STATUS_ABORTED:
            status = self.on_aborted(result)
        else:
            self.node.get_logger().error('[{}] invalid GoalStatus:{} after get_result from server'.format(self.name, goal_status))
            return Status.FAILURE            
                
        return status
    
    def send_new_goal(self):
        self.send_goal_future = self.action_client.send_goal_async(self.goal, feedback_callback=self.feedback_cb)
        self.sent_time = self.node.get_clock().now()
        
    def is_send_goal_future_complete(self, elapsed_time: float, timeout: float):
        
        if elapsed_time > self.server_timeout:
            self.send_goal_future = None
            #server timeout
            return False
        
        self.node.executor.spin_once(timeout)
        send_goal_exception = self.send_goal_future.exception()
        
        if send_goal_exception:
            self.node.get_logger().warn('[{}] send_goal_future got exception while sending goal : {}'.format(self.name, send_goal_exception))
            raise RuntimeError("Sending Goal Failed")
        
        if self.send_goal_future.done():
            self.goal_handle = self.send_goal_future.result()
            self.send_goal_future = None
            if not self.goal_handle.accepted:
                raise RuntimeError("Goal was rejected by the action server")
            self.get_result_future:rclpy.Future = self.goal_handle.get_result_async()
            self.get_result_future.add_done_callback(self.result_cb)
            return True
        
        return False       
    
    def feedback_cb(self, feedback_msg):
        # goal_id = feedback_msg.goal_id
        self.action_feedback = feedback_msg.feedback
        
    def result_cb(self, future):
        if self.send_goal_future:
            self.node.get_logger().warn("[{}] Goal result for {} available, but it hasn't received the goal response yet.\
                It's probably a goal result for the last goal request".format(self.name, self.action_topic))
            return
        
        result_message:ResultMessage = future.result()
        
        if self.goal_handle:
            if all(self.goal_handle.goal_id.uuid == result_message.goal_id):
                self.result = result_message
                self.result_avaliable = True
            else:
                self.node.get_logger().warn("[{}] Goal result for {} available, but result goal_id\
                    not match with current goal handle\n{}:{}".format(self.name, self.action_topic, self.goal_handle.goal_id.uuid, result_message.goal_id))
                    
    @abstractmethod
    def on_update(self) -> typing.Tuple[bool, ActionGoal]:
        """
        trigger once when transition from any BtStatus to BtStatus.RUNNING
        implement btActionNode.on_update logic for create action_goal and whether to send_goal or not

        :raises NotImplementedError: this method must be implement in derived class
        :return: (should_sent_goal, goal) should_sent_goal flag set to True to send_goal to Server
        :rtype: typing.Tuple[bool, ActionGoal]
        """
        raise NotImplementedError
    
    @abstractmethod
    def on_success(self, result:ActionResult) -> Status:
        return Status.SUCCESS
    
    @abstractmethod
    def on_aborted(self, result:ActionResult) -> Status:
        return Status.FAILURE
    
    @abstractmethod
    def on_cancelled(self, result:ActionResult) -> Status:
        return Status.SUCCESS
    
    @abstractmethod
    def on_wait_for_result(self, feedback: ActionFeedback):
        pass
    
    def terminate(self, new_status: Status) -> None:
        if self.should_cancel_goal():
            # action_server always cancel accepted 
            self.goal_handle.cancel_goal_async()
            
    
    def should_cancel_goal(self):
        if self.status != Status.RUNNING:
            return False
        
        if not self.goal_handle:
            return False
        
        return self.goal_handle.status == GoalStatus.STATUS_ACCEPTED or self.goal_handle.status == GoalStatus.STATUS_EXECUTING
        