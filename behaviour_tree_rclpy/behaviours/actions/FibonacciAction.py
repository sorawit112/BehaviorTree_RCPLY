import typing

from py_trees.common import Status
from behaviour_tree_rclpy.base_behaviour.bt_action_node import BtActionNode
from example_interfaces.action import Fibonacci
import threading 


class FibonacciAction(BtActionNode):
    def __init__(self, attr:dict={}):
        name: str = attr['name'] if 'name' in attr.keys() else 'FibonacciAction'
        action_topic: str = attr['action_topic']
        send_goal_timeout: float = float(attr['send_goal_timeout'])
        server_timeout: float = float(attr['server_timeout']) 
        order: int = int(attr['order'])
        
        self.lock = threading.Lock
                
        super().__init__(name, action_topic, Fibonacci, send_goal_timeout, server_timeout)
        
        self.order = order
        
    def on_update(self) -> typing.Tuple[bool, Fibonacci.Goal]:
        goal = Fibonacci.Goal()
        goal.order = self.order
        return True, goal
    
    def on_wait_for_result(self, feedback: Fibonacci.Feedback):
        if feedback:
            self.feedback_message = "Feedback : {}".format(feedback.sequence)
        else:
            self.feedback_message = "Feedback : None"
        return super().on_wait_for_result(feedback)
    
    def on_success(self, result: Fibonacci.Result) -> Status:
        self.feedback_message = "Success : {}".format(result.sequence)
        return super().on_success(result)
    
    def on_aborted(self, result: Fibonacci.Result) -> Status:
        return super().on_aborted(result)
    
    def on_cancelled(self, result: Fibonacci.Result) -> Status:
        return super().on_cancelled(result)
        
if __name__ == "__main__":
    attr = {'service_topic': 'temp', 'server_timeout': 8, 'send_goal_timeout':1}
    trigger_service = Fibonacci(attr)