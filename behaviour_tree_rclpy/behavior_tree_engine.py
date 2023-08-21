import rclpy

from rclpy.task import Future
from rclpy.node import Node
from enum import Enum
from typing import Union, Callable
import py_trees
from py_trees.trees import BehaviourTree
from py_trees.behaviour import Behaviour
from py_trees.common import Status as NodeStatus
from behaviour_tree_rclpy.bt_factory import BehaviourTreeFactory

class BtStatus(Enum):
    SUCCEED = 0
    CANCELED = 1
    FAILED = 2


class BehaviourTreeEngine:
    def __init__(self):
        self.factory_ = BehaviourTreeFactory()
        
    async def run(
        self,
        node: Node,
        tree: BehaviourTree, 
        onloop: Callable, 
        cancel_request: Callable, 
        done_cb: Callable,
    ):
        
        bt_result_future = Future()
        bt_result_future.add_done_callback(done_cb)
        
        result = NodeStatus.RUNNING
        
        try:
            if rclpy.ok() and node.context.ok() and result == NodeStatus.RUNNING:
                if cancel_request():
                    self.halt_all_actions(tree)
                    return BtStatus.CANCELED

                tree.tick()
                node.get_logger().info('\n{}'.format(
                    py_trees.display.unicode_tree(root=tree.root, show_status=True)))
                
                result = tree.root.status
                
                onloop()
    
        except Exception as e:
            node.get_logger().error("Behavior tree threw exception: {}. Exiting with failure.".format(e))

            bt_result_future.set_result(BtStatus.FAILED)
        
        if result != NodeStatus.RUNNING:
            result_future = BtStatus.SUCCEED if result == NodeStatus.SUCCESS else BtStatus.FAILED
            bt_result_future.set_result(result_future)
        
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