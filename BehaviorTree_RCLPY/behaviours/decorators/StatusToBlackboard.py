from py_trees.behaviour import Behaviour
from py_trees.decorators import StatusToBlackboard as pyStatusToBlackboard

class StatusToBlackboard(pyStatusToBlackboard):
    def __init__(
        self, 
        name: str, 
        child: Behaviour, 
        variable_name: str
    ):
        super().__init__(name, child, variable_name)

