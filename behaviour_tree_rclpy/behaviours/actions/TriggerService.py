import typing

from py_trees.common import Status
from behaviour_tree_rclpy.base_behaviour.bt_service_node import BtServiceNode, SrvTypeRequest, SrvTypeResponse
from std_srvs.srv import Trigger


class TriggerService(BtServiceNode):
    def __init__(self, attr:dict={}):
        name: str = attr['name'] if 'name' in attr.keys() else 'TriggerService'
        service_topic: str = attr['service_topic']
        server_timeout: float = float(attr['server_timeout']) 
        
        super().__init__(name, service_topic, Trigger, server_timeout)
        
    def on_update(self) -> typing.Tuple[bool, Trigger.Request]:
        trigger_request = Trigger.Request()
        return True, trigger_request
    
    def on_wait_for_result(self):
        return super().on_wait_for_result()
    
    def on_completion(self, response: Trigger.Response) -> Status:
        self.feedback_message = f"Server message : {response.message}"
        if response.success:
            return Status.SUCCESS
        else:
            return Status.FAILURE
        
if __name__ == "__main__":
    attr = {'service_topic': 'temp', 'server_timeout': 1}
    trigger_service = TriggerService(attr)