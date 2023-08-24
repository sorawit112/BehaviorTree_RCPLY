
import typing
from abc import ABC, abstractmethod

import rclpy
from rclpy.time import Time
from rclpy.node import Node
from rclpy.callback_groups import CallbackGroup
from rclpy.service import SrvType, SrvTypeRequest, SrvTypeResponse

from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access


to_sec = 1e-9

class BtServiceNode(Behaviour, ABC):
    loop_safty_factor = 0.9
    def __init__(
        self, 
        name: str, 
        service_topic: str, 
        service_type: SrvType,
        server_timeout: float=1.0,
    ):
        super().__init__(name)
        
        self.blackboard = self.attach_blackboard_client(name)
        self.blackboard.register_key('node', access=Access.READ, required=True)
        self.blackboard.register_key('bt_loop_duration', access=Access.READ, required=True)
        self.blackboard.register_key('service_cb_group', access=Access.READ, required=True)
        
        self.node: Node = self.blackboard.node
        self.service_type: SrvType = service_type
        self.service_topic: str = service_topic
        self.bt_loop_duration: float = self.blackboard.bt_loop_duration * self.loop_safty_factor
        self.server_timeout: float = server_timeout
        self.cb_group: CallbackGroup = self.blackboard.service_cb_group
        
        self.sent_time: typing.Optional[Time] = None
        self.request_sent: bool = False
        self.future_result: typing.Optional[rclpy.Future] = None
        
    def setup(self):
        self.service_client = self.node.create_client(self.service_type, 
                                self.service_topic, callback_group=self.cb_group)

        self.node.get_logger().info(
            '[{}] Waiting for {} service]'.format(self.name, self.service_topic))
        
        if not self.service_client.wait_for_service(1.0):
            self.node.get_logger().debug(
                '[{}] {} service server not available after waiting for 1 s'.format(self.name, self.service_topic))

            raise RuntimeError('Service server not available')
        
        self.node.get_logger().info('[{}] BtServiceNode initialized]'.format(self.name))
        
    def initialise(self) -> None:
        self.request_sent = False
        self.future_result = None
    
    def update(self) -> Status:
        if not self.request_sent:
            should_sent_request, request = self.on_update()
            
            if not should_sent_request:
                return Status.FAILURE
            
            self.future_result = self.service_client.call_async(request)
            self.sent_time = self.node.get_clock().now()
            self.request_sent = True
            
        return self.check_future()
        
    @abstractmethod
    def on_update(self) -> typing.Tuple[bool, SrvTypeRequest]:
        """
        trigger once when transition from any BtStatus to BtStatus.RUNNING
        implement btservicenode logic for create service_request and whether to send_request or not

        :raises NotImplementedError: this method must be implement in derived class
        :return: (should_send_request, request) should_send_request flag set to True to send service_request to Server
        :rtype: typing.Tuple[bool, SrvTypeRequest]
        """
        raise NotImplementedError
    
    def check_future(self) -> Status:
        elapsed_time = (self.node.get_clock().now() - self.sent_time).nanoseconds*to_sec
        remaining_time = self.server_timeout - elapsed_time
        
        if remaining_time > 0:
            timeout = remaining_time if self.bt_loop_duration > remaining_time else self.bt_loop_duration
            self.node.executor.spin_once(timeout)
            if self.future_result.done():
                self.request_sent = False
                status = self.on_completion(self.future_result.result())
                return status
            else:
                self.on_wait_for_result()
                elapsed_time = (self.node.get_clock().now() - self.sent_time).nanoseconds*to_sec
                if elapsed_time < self.server_timeout:
                    return Status.RUNNING
        
        self.request_sent = False
        self.feedback_message = f"server_timeout : {self.server_timeout} sec"
        self.node.get_logger().debug(
            '[{}] server_timeout while waiting for result from {}'.format(self.name, self.service_topic))
        return Status.FAILURE
    
    @abstractmethod
    def on_completion(self, response: SrvTypeResponse) -> Status:
        return Status.SUCCESS
    
    @abstractmethod
    def on_wait_for_result(self, feedback_message=""): 
        self.feedback_message = f"wait for result : {feedback_message}"
    
    def terminate(self, new_status: Status) -> None:
        return super().terminate(new_status)    