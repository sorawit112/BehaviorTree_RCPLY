
import rclpy
from rclpy.task import Future
from rclpy.node import Node
from behaviour_tree_rclpy.behavior_tree_engine import BehaviourTreeEngine, BtStatus
from behaviour_tree_rclpy.bt_factory import DEFAULT_XML

class BehaviourTreeNode(Node):
    def __init__(self, bt_xml=None) -> None:
        super().__init__("behaviour_tree")
        self.bt_xml_file = bt_xml if bt_xml else DEFAULT_XML
        
        self.bt_ = BehaviourTreeEngine()
        self.tree = self.bt_.create_tree_from_xml_file(self.bt_xml_file)
        
        self.loop_duration = 1 #seconds
        
        self.execute_timer = self.create_timer(self.loop_duration, self.execute)
        
        self.get_logger().info('intialized')
        
    def bt_result_cb(self, future:Future):
        self.get_logger().info('bt_result_cb')
        self.execute_timer.destroy()
        result = future.result()
        
        if result == BtStatus.SUCCEED:
            self.get_logger().info('behavior_tree succeed')
        elif result == BtStatus.CANCELED:
            self.get_logger().info('behavior_tree canceled')
        elif result == BtStatus.FAILED:
            self.get_logger().info('behavior_tree failed')
        else:
            raise ValueError('Invalid behavior_tree Result : {}'.format(result))
        
    async def execute(self):
        self.get_logger().info('tick')
        result = await self.bt_.run(self, self.tree, self.onloop, self.cancel_request, self.bt_result_cb)
        print(result)
        
    def onloop(self):
        self.get_logger().info('onloop')
        
    def cancel_request(self):
        self.get_logger().info('cancel_request()')
        return False
    
    
def main():
    rclpy.init()
    
    node = BehaviourTreeNode()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception) as e:
        print(e)
        node.destroy_node()
    finally:
        rclpy.shutdown()
        

if __name__=="__main__":
    main()
        

