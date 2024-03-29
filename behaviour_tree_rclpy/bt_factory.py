from ament_index_python.packages import get_package_share_directory
import typing 
import xml.etree.ElementTree as ET
import copy
import importlib.util
import inspect
import ast

from py_trees.trees import BehaviourTree
from py_trees.behaviour import Behaviour
from behaviour_tree_rclpy.behaviours.actions import AlwaysFailure, \
    AlwaysRunning, AlwaysSuccess, BlackboardToStatus, Periodic, \
    SetBlackboard, SetBlackboardVariable, StatusQueue, TickCounter, \
    Timer, UnsetBlackboardVariable, WaitForBlackboardVariable, TriggerService, FibonacciAction
from behaviour_tree_rclpy.behaviours.conditions import CheckBlackboardVariableExists, CheckBlackboardVariableValue, SuccessEveryN
from behaviour_tree_rclpy.behaviours.controls import Fallback, FallbackStar, Parallel, ParallelPolicy, Sequence, SequenceStar
from behaviour_tree_rclpy.behaviours.decorators import Condition, Count, FailureIsRunning, FailureIsSuccess, \
    ForceFailure, ForceSuccess, Inverter, KeepRunningUntilFailure, OneShot, PassThrough, \
    Repeat, RetryUntilSuccessful, RunningIsFailure, RunningIsSuccess, StatusToBlackboard, \
    SuccessIsFailure, SuccessIsRunning, Timeout

DEFAULT_XML = get_package_share_directory('behaviour_tree_rclpy') + '/default_tree.xml'

default_action_type_mapping = {
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
    'TriggerService' : TriggerService.TriggerService,
    'FibonacciAction' : FibonacciAction.FibonacciAction,
}

default_condition_type_mapping = {
    'CheckBlackboardVariableExists' : CheckBlackboardVariableExists.CheckBlackboardVariableExists,
    'CheckBlackboardVariableValue' : CheckBlackboardVariableValue.CheckBlackboardVariableValue,
    'SuccessEveryN' : SuccessEveryN.SuccessEveryN
}

default_control_type_mapping = {
    'Fallback' : Fallback.Fallback,
    'FallbackStar' : FallbackStar.FallbackStar,
    'Parallel' : Parallel.Parallel,
    'ParallelPolicy' : ParallelPolicy.ParallelPolicy,
    'Sequence' : Sequence.Sequence,
    'SequenceStar' : SequenceStar.SequenceStar,
}

default_decorator_type_mapping = {
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
}

class BehaviourTreeFactory:
    def __init__(
        self, 
        xml_file : typing.Optional[str]=None
    ):
        self.xml_file = xml_file if xml_file else DEFAULT_XML
        
        self.action_type_mapping = default_action_type_mapping
        self.control_type_mapping = default_control_type_mapping
        self.condition_type_mapping = default_condition_type_mapping
        self.decorator_type_mapping = default_decorator_type_mapping
        
        self.node_type_mapping = node_type_mapping
        self.node_type_mapping['Action'] = self.action_type_mapping
        self.node_type_mapping['Condition'] = self.condition_type_mapping
        self.node_type_mapping['Control'] = self.control_type_mapping
        self.node_type_mapping['Decorator'] = self.decorator_type_mapping
        
    def __str__(self):
        str_ = "****bt_factory behaviors type mapping lists****\n"
        str_ += "--------- action_type_mapping ----------\n"
        for key,value in self.action_type_mapping.items():
            str_ += f"    '{key}' : [{value.__module__}]\n"
        str_ += "--------- control_type_mapping ----------\n"
        for key,value in self.control_type_mapping.items():
            str_ += f"    '{key}' : [{value.__module__}]\n"
        str_ += "--------- condition_type_mapping ----------\n"
        for key,value in self.condition_type_mapping.items():
            str_ += f"    '{key}' : [{value.__module__}]\n"
        str_ += "--------- decorator_type_mapping ----------\n"
        for key,value in self.decorator_type_mapping.items():
            str_ += f"    '{key}' : [{value.__module__}]\n"
        str_ += "***********************************************\n\n"
        
        return str_
    
    def load_action_type_from_module(self, package_name):
        action_module = __import__(package_name).behaviors.Action
        classes = self.get_exclude_class(action_module)
        for cls_name, cls in classes:
            self.add_action_type_mapping(cls_name, cls)
            
    def load_condition_type_from_module(self, package_name):
        condition_module = __import__(package_name).behaviors.Condition
        
        classes = self.get_exclude_class(condition_module)
        for cls_name, cls in classes:
            self.add_condition_type_mapping(cls_name, cls)
                    
    def get_exclude_class(self, module) -> typing.Tuple[str, type]:
        all_classes = inspect.getmembers(module, inspect.isclass)
        exclude_class = []
        
        spec = importlib.util.find_spec(module.__name__)
        if spec is None:
            raise ImportError(f"Module '{module.__name__}' not found")

        with open(spec.origin, 'r') as file:
            file_content = file.read()
            tree = ast.parse(file_content)

            # Get imported modules from the file
            imports = [node.names[0].name for node in ast.walk(tree) if isinstance(node, ast.Import) for alias in node.names]
            imports.extend([node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)])
            
            excluded_classe_name = [node.name for node in ast.walk(tree) if isinstance(node, ast.ClassDef) and node.name not in imports]
            
            exclude_class.extend([(cls_name, cls) for cls_name, cls in all_classes if cls_name in excluded_classe_name])
            
            return exclude_class
            
    def add_action_type_mapping(self, key, value):
        self.action_type_mapping[key] = value
        
    def add_control_type_mapping(self, key, value):
        self.control_type_mapping[key] = value
        
    def add_condition_type_mapping(self, key, value):
        self.condition_type_mapping[key] = value
        
    def add_decorator_type_mapping(self, key, value):
        self.decorator_type_mapping[key] = value
        
    def registerNodeType(self, node_type:str, **kwargs):
        try:
            node_attrb = kwargs['node_attrb']
            if node_type in ['Action', 'Condition', 'Control', 'Decorator']:
                node = node_type_mapping[node_type][node_attrb['ID']]
                node_type = node_attrb['ID']
            else:
                node = node_type_mapping[node_type]
            return node, node_type
        except KeyError as er:
            print(f'KeyError : {er}')
            return None, node_type
        
    def create_node(
        self, 
        xml_element:ET.Element, 
        sub_trees:dict,
        **kwargs
    ) -> Behaviour:
        node_type = xml_element.tag
        node = None

        if node_type == "BehaviorTree":
            node = self.create_node(xml_element[0], sub_trees, **kwargs)
        elif node_type == "SubTree":
            sub_tree_id = xml_element.get("ID")
            if sub_tree_id in sub_trees:
                sub_tree_root = sub_trees[sub_tree_id]
                if 'sub_tree_attrib' in kwargs.keys():
                    sub_tree_attrib = self.remapping_attrb(xml_element.attrib, **kwargs)
                else:
                    sub_tree_attrib = copy.deepcopy(xml_element.attrib)
                node = self.create_node(sub_tree_root, sub_trees, sub_tree_attrib=sub_tree_attrib)
            else:
                raise KeyError(f"sub_tree:{sub_tree_id} not in {sub_trees.keys()}")
        else:
            node, node_type = self.registerNodeType(node_type, node_attrb=xml_element.attrib)
            
            if 'sub_tree_attrib' in kwargs.keys():
                node_attr = self.remapping_attrb(xml_element.attrib, **kwargs)
            else:
                node_attr = copy.deepcopy(xml_element.attrib)
            
            children = []
            for child_element in xml_element:
                child_node = self.create_node(child_element, sub_trees, **kwargs)
                if child_node:
                    children.append(child_node)
            
            if len(children) > 0:
                if node_type in self.decorator_type_mapping:
                    return node(children[0], node_attr)
                elif node_type in self.control_type_mapping:
                    return node(children, node_attr)
                elif node_type in self.action_type_mapping or node_type in self.condition_type_mapping:
                    raise ValueError(f"{node_type} can't have child : {children}")
                else:
                    raise KeyError(f"Can't mapping key:{node_type}")
            else:
                return node(node_attr)

        return node     
    
    def remapping_attrb(self, node_attrib:dict, sub_tree_attrib:dict):
        attr = copy.deepcopy(node_attrib)
        str_ = ""
        for key, value in node_attrib.items():
            if value in sub_tree_attrib.keys():
                attr[key] = sub_tree_attrib[value]
                str_ += f"  >> Found remapping attribute in sub_tree_attrb['{value}'] : {sub_tree_attrib[value]}\n"
                str_ += f"  >> Remapping node_attrib['{key}'] : {value} --> {sub_tree_attrib[value]}\n"
        
        if str_ != "" and self.debug:
            str_ = f"-----------{node_attrib.get('ID')}-----------\n" + str_ + '-----------------------------'
            print(str_)
        return attr
           
    def load_behavior_tree_from_xml(
        self, 
        xml_file_path:typing.Optional[str]=None,
        debug = False
    ) -> typing.Optional[Behaviour]:
        self.debug = debug
        if self.debug:
            print('load_behavior_tree_from_xml')
        if not xml_file_path:
            xml_file_path = self.xml_file
            
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
    print(factory)
    # xml = get_package_share_directory('behaviour_tree_rclpy') + '/test_behaviour.xml'
    # behavior_tree_root = factory.load_behavior_tree_from_xml(xml)

    # py_trees.logging.level = py_trees.logging.Level.DEBUG
    
    # if behavior_tree_root:
    #     behavior_tree = BehaviourTree(behavior_tree_root)
    #     py_trees.display.render_dot_tree(behavior_tree_root)
    #     for i in range(10):
    #         print("\n--------- Tick {0} ---------\n".format(i))
    #         behavior_tree.tick()
    #         print("\n")
    #         print(py_trees.display.unicode_tree(root=behavior_tree_root, show_status=True))
    #         time.sleep(1.0)
    # else:
    #     print("Error: Unable to create behavior tree.")