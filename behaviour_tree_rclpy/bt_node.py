
import rclpy
from rclpy.task import Future
from rclpy.node import Node
from py_trees.common import Status as NodeStatus
from behaviour_tree_rclpy.behavior_tree_engine import BehaviourTreeEngine, BtStatus
from behaviour_tree_rclpy.bt_factory import DEFAULT_XML
from ament_index_python import get_package_share_directory

class BehaviourTreeNode(Node):
    def __init__(self, bt_xml=None) -> None:
        super().__init__("behaviour_tree")
        self.bt_xml_file = bt_xml if bt_xml else DEFAULT_XML
        self.bt_loop_duration = 1 #seconds
        
        self.bt_ = BehaviourTreeEngine(self)
        self.tree = self.bt_.create_tree_from_xml_file(self.bt_xml_file)
        self.bt_.setup_all_actions(self.tree)
        
        self.execute_timer = self.create_timer(self.bt_loop_duration, self.execute)
        
        self.get_logger().info('intialized')
        
    def bt_result_cb(self, result):
        self.get_logger().info('bt_result_cb')
        self.execute_timer.destroy()
        
        if result == BtStatus.SUCCEED:
            self.get_logger().info('behavior_tree succeed')
        elif result == BtStatus.CANCELED:
            self.get_logger().info('behavior_tree canceled')
        elif result == BtStatus.FAILED:
            self.get_logger().info('behavior_tree failed')
        else:
            raise ValueError('Invalid behavior_tree Result : {}'.format(result))
        
    async def execute(self):
        self.get_logger().debug('executing')
        if rclpy.ok() and self.context.ok() and not self.cancel_request():
            result = await self.bt_.run(self.tree)
            self.onloop()
        if result != NodeStatus.RUNNING:
            self.bt_result_cb(result)
        
    def onloop(self):
        self.get_logger().debug('onloop :)')
        
    def cancel_request(self):
        self.get_logger().debug('cancel_request()')
        return False
    
    
def main(args=None):
    rclpy.init(args=args)
    
    xml = get_package_share_directory('behaviour_tree_rclpy') + '/test_behaviour.xml'
    node = BehaviourTreeNode(xml)
    try:
        rclpy.spin(node)
        
    except (KeyboardInterrupt, Exception) as e:
        print(e)
        node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        else:
            print('\n\nrclpy already shutdown')
        

if __name__=="__main__":
    main()
        

