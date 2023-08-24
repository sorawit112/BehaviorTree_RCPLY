from ament_index_python.packages import get_package_share_directory
import typing 
import xml.etree.ElementTree as ET

from py_trees.trees import BehaviourTree
from py_trees.behaviour import Behaviour
from behaviour_tree_rclpy.behaviours.actions import AlwaysFailure, \
    AlwaysRunning, AlwaysSuccess, BlackboardToStatus, Periodic, \
    SetBlackboard, SetBlackboardVariable, StatusQueue, TickCounter, \
    Timer, UnsetBlackboardVariable, WaitForBlackboardVariable, TriggerService
from behaviour_tree_rclpy.behaviours.conditions import CheckBlackboardVariableExists, CheckBlackboardVariableValue, SuccessEveryN
from behaviour_tree_rclpy.behaviours.controls import Fallback, FallbackStar, Parallel, ParallelPolicy, Sequence, SequenceStar
from behaviour_tree_rclpy.behaviours.decorators import Condition, Count, FailureIsRunning, FailureIsSuccess, \
    ForceFailure, ForceSuccess, Inverter, KeepRunningUntilFailure, OneShot, PassThrough, \
    Repeat, RetryUntilSuccessful, RunningIsFailure, RunningIsSuccess, StatusToBlackboard, \
    SuccessIsFailure, SuccessIsRunning, Timeout

DEFAULT_XML = get_package_share_directory('behaviour_tree_rclpy') + '/default_tree.xml'

action_type_mapping = {
    'AlwaysFailure' : AlwaysFailure.AlwaysFailure,
    'AlwaysRunning' : AlwaysRunning.AlwaysRunning,
    'AlwaysSuccess' : AlwaysSuccess.AlwaysSuccess,
    'BlackboardToStatus' : BlackboardToStatus.BlackboardToStatus,
    'Periodic' : Periodic.Periodic,
    'SetBlackboard' : SetBlackboard.SetBlackboard,
    'SetBlackboardVariable' : SetBlackboardVariable.SetBlackboardVariable,
    'StatusQueue' : StatusQueue.StatusQueue,
    'TickCounter' : TickCounter.TickCounter,
    'Timer' : Timer.Timer,
    'UnsetBlakboardVariable' : UnsetBlackboardVariable.UnsetBlackboardVariable,
    'WaitForBlackboardVariable' : WaitForBlackboardVariable.WaitForBlackboardVariable,
    'TriggerService' : TriggerService.TriggerService
}

condition_type_mapping = {
    'CheckBlackboardVariableExists' : CheckBlackboardVariableExists.CheckBlackboardVariableExists,
    'CheckBlackboardVariableValue' : CheckBlackboardVariableValue.CheckBlackboardVariableValue,
    'SuccessEveryN' : SuccessEveryN.SuccessEveryN
}

control_type_mapping = {
    'Fallback' : Fallback.Fallback,
    'FallbackStar' : FallbackStar.FallbackStar,
    'Parallel' : Parallel.Parallel,
    'ParallelPolicy' : ParallelPolicy.ParallelPolicy,
    'Sequence' : Sequence.Sequence,
    'SequenceStar' : SequenceStar.SequenceStar,
}

decorator_type_mapping = {
    'Condition' : Condition.Condition,
    'Count' : Count.Count,
    'FailureIsRunning' : FailureIsRunning.FailureIsRunning,
    'FailureIsSuccess' : FailureIsSuccess.FailureIsSuccess,
    'ForceFailure' : ForceFailure.ForceFailure,
    'ForceSuccess' : ForceSuccess.ForceSuccess,
    'Inverter' : Inverter.Inverter,
    'KeepRunningUntilFailure' : KeepRunningUntilFailure.KeepRunningUntilFailure,
    'Oneshot' : OneShot.OneShot,
    'PassThrough' : PassThrough.PassThrough,
    'Repeat' : Repeat.Repeat,
    'RetryUntilSuccessful' : RetryUntilSuccessful.RetryUntilSuccessful,
    'RunningIsFailure' : RunningIsFailure.RunningIsFailure,
    'RunningIsSuccess' : RunningIsSuccess.RunningIsSuccess,
    'StatusToBlackboard' : StatusToBlackboard.StatusToBlackboard,
    'SuccessIsFailure' : SuccessIsFailure.SuccessIsFailure,
    'SuccessIsRunning' : SuccessIsRunning.SuccessIsRunning,
    'Timeout' : Timeout.Timeout
}

node_type_mapping = {
    'AlwaysFailure' : AlwaysFailure.AlwaysFailure,
    'AlwaysSuccess' : AlwaysSuccess.AlwaysSuccess,
    'SetBlackboard' : SetBlackboard.SetBlackboard,
    'Fallback' : Fallback.Fallback,
    'Parallel' : Parallel.Parallel,
    'Sequence' : Sequence.Sequence,
    'SequenceStar' : SequenceStar.SequenceStar,
    'ForceFailure' : ForceFailure.ForceFailure,
    'ForceSuccess' : ForceSuccess.ForceSuccess,
    'Inverter' : Inverter.Inverter,
    'KeepRunningUntilFailure' : KeepRunningUntilFailure.KeepRunningUntilFailure,
    'Repeat' : Repeat.Repeat,
    'RetryUntilSuccessful' : RetryUntilSuccessful.RetryUntilSuccessful,
    'Timeout' : Timeout.Timeout,    
    'Action' : action_type_mapping,
    'Condition' : condition_type_mapping,
    'Control' : control_type_mapping,
    'Decorator' : decorator_type_mapping,
}

class BehaviourTreeFactory:
    def __init__(
        self, 
        xml_file : typing.Optional[str]=None
    ):
        self.xml_file = xml_file if xml_file else DEFAULT_XML
        
    def registerNodeType(self, node_type:str, **kwargs):
        try:
            kwargs = kwargs['kwargs']
            if node_type in ['Action', 'Condition', 'Control', 'Decorator']:
                node = node_type_mapping[node_type][kwargs['ID']]
                node_type = kwargs['ID']
            else:
                node = node_type_mapping[node_type]
            return node, node_type
        except KeyError as er:
            print(er)
            return None, node_type
        
    def create_node(
        self, 
        xml_element:ET.Element, 
        sub_trees:dict
    ) -> Behaviour:
        node_type = xml_element.tag
        node = None

        if node_type == "BehaviorTree":
            node = self.create_node(xml_element[0], sub_trees)
        elif node_type == "SubTree":
            sub_tree_id = xml_element.get("ID")
            if sub_tree_id in sub_trees:
                sub_tree_root = sub_trees[sub_tree_id]
                node = self.create_node(sub_tree_root, sub_trees)
            else:
                raise KeyError(f"sub_tree:{sub_tree_id} not in {sub_trees.keys()}")
        else:
            node, node_type = self.registerNodeType(node_type, kwargs=xml_element.attrib)
            
            children = []
            for child_element in xml_element:
                child_node = self.create_node(child_element, sub_trees)
                if child_node:
                    children.append(child_node)
                    
            if len(children) > 0:
                if node_type in decorator_type_mapping:
                    return node(children[0], xml_element.attrib)
                elif node_type in control_type_mapping:
                    return node(children, xml_element.attrib)
                elif node_type in action_type_mapping or node_type in condition_type_mapping:
                    raise ValueError(f"{node_type} can't have child : {children}")
                else:
                    raise KeyError(f"Can't mapping key:{node_type}")
            else:
                return node(xml_element.attrib)

        return node        
        
    def load_behavior_tree_from_xml(
        self, 
        xml_file_path:typing.Optional[str]=None
    ) -> typing.Optional[Behaviour]:
        if not xml_file_path:
            xml_file_path = DEFAULT_XML
            
        tree = ET.parse(xml_file_path)
        root_element = tree.getroot()
        main_tree_id = root_element.get("main_tree_to_execute")
        
        sub_trees = {}
        for element in root_element:
            element_id = element.get("ID")
            if element_id and element.tag != 'TreeNodesModel':
                sub_trees[element_id] = element

        for element in root_element.iter():
            if element.get("ID") == main_tree_id:
                return self.create_node(element, sub_trees)

        return None
    
if __name__=="__main__":
    import py_trees, time
    
    factory = BehaviourTreeFactory()
    xml = get_package_share_directory('behaviour_tree_rclpy') + '/test_behaviour.xml'
    behavior_tree_root = factory.load_behavior_tree_from_xml(xml)

    py_trees.logging.level = py_trees.logging.Level.DEBUG
    
    if behavior_tree_root:
        behavior_tree = BehaviourTree(behavior_tree_root)
        py_trees.display.render_dot_tree(behavior_tree_root)
        for i in range(10):
            print("\n--------- Tick {0} ---------\n".format(i))
            behavior_tree.tick()
            print("\n")
            print(py_trees.display.unicode_tree(root=behavior_tree_root, show_status=True))
            time.sleep(1.0)
    else:
        print("Error: Unable to create behavior tree.")