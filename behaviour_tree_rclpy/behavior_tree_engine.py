import rclpy

from rclpy.task import Future
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from enum import Enum
from typing import Union, Callable
import py_trees
from py_trees.trees import BehaviourTree
from py_trees.behaviour import Behaviour
from py_trees.common import Status as NodeStatus
from py_trees.common import Access
from behaviour_tree_rclpy.bt_factory import BehaviourTreeFactory

class BtStatus(Enum):
    SUCCEED = 0
    CANCELED = 1
    FAILED = 2


class BehaviourTreeEngine:
    module_name = 'engine'
    def __init__(self, node:Node):
        self.node = node
        self.factory_ = BehaviourTreeFactory()
        self.blackboard = self.initialize_blackboard()
        
        self.node.get_logger().info('[{}] initialize'.format(self.module_name))
        
    def initialize_blackboard(self):
        blackboard = py_trees.blackboard.Client(name="engine")
        blackboard.register_key('node', access=Access.EXCLUSIVE_WRITE)
        blackboard.register_key('bt_loop_duration', access=Access.EXCLUSIVE_WRITE)
        blackboard.register_key('service_cb_group', access=Access.EXCLUSIVE_WRITE)
        blackboard.register_key('action_cb_group', access=Access.EXCLUSIVE_WRITE)
        blackboard.node = self.node
        blackboard.bt_loop_duration = self.node.bt_loop_duration
        blackboard.service_cb_group = MutuallyExclusiveCallbackGroup()
        blackboard.action_cb_group = MutuallyExclusiveCallbackGroup()
        
        return blackboard
        
    async def run(self, tree: BehaviourTree):
        result = NodeStatus.RUNNING
        
        try:
            tree.tick()
            self.node.get_logger().info('\n{}'.format(
                py_trees.display.unicode_tree(root=tree.root, show_status=True)))
            
            result = tree.root.status
    
        except Exception as e:
            self.node.get_logger().error(
                "[{}] Behavior tree threw exception: {}. Exiting with failure.".format(self.module_name,e))
                
        return result
        
    def create_tree_from_xml_file(self, xml_file:str):
        """
        create wrapper of bt_factory to create BehavioTree

        :param xml_file: path to behaviour_tree xml_file
        :type xml_file: str
        :return: behaviour_tree object from py_trees library
        :rtype: BehaviourTree
        """
        root_node = self.factory_.load_behavior_tree_from_xml(xml_file)
        root_node.setup()
        return BehaviourTree(root_node)
    
    def halt_all_actions(self, tree: Union[BehaviourTree, Behaviour]):
        """
        stop all node in tree and set to INVALID status

        :param tree: BehaviourTree or Behaviour Node from Tree
        :type tree: Union[BehaviourTree, Behaviour]
        :raises TypeError: 
        """
        if isinstance(tree, BehaviourTree):
            root_node = tree.root
        elif isinstance(tree, Behaviour):
            root_node = tree
        else:
            raise TypeError('input tree must be BehaviourTree or Behaviour(root_node)')
        
        if not root_node:
            print('root_node is None skipping halt action')
            return
        
        for node in tree.root.iterate():
            node.stop(NodeStatus.INVALID)