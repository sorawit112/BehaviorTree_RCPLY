from typing import List
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from threading import Event
from random import random, randint

class TriggerServer(Node):
    def __init__(self) -> None:
        super().__init__("tigger_server")
        
        self.create_service(Trigger, 'trigger', self.trigger_cb)
        self.trigger = False
        self.event = Event()
        
        self.get_logger().info(f'initiate trigger as : {self.trigger}')
        
    def trigger_cb(self, request: Trigger.Request, response: Trigger.Response):
        self.trigger = not self.trigger
        delay_time = random() + randint(0,1)*0.7
        self.get_logger().info('get service request delay_time: {:.2f}'.format(delay_time))
        self.event.wait(delay_time)
        self.event.clear()
        self.get_logger().info(f'trigger to : {self.trigger}\n')
        
        response.success = True
        response.message = f'trigger to {self.trigger}'
        
        return response
        
def main():
    rclpy.init()
    node = TriggerServer()
    try:
        rclpy.spin(node)
    except:
        node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        else:
            print('\n\nrclpy already shutdown')
        