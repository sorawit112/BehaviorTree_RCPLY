from py_trees.behaviour import Behaviour
from py_trees.decorators import StatusToBlackboard as pyStatusToBlackboard

class StatusToBlackboard(pyStatusToBlackboard):
    def __init__(
        self, 
        child: Behaviour,
        attr:dict={},
    ):
        name: str = attr['name'] if 'name' in attr.keys() else 'RunningIsSuccess'
        variable_name: str = attr['variable_name']
        super().__init__(name, child, variable_name)

