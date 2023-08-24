from py_trees.behaviour import Behaviour
from py_trees.decorators import RunningIsFailure as pyRunningIsFailure


class RunningIsFailure(pyRunningIsFailure):
    def __init__(
        self, 
        child: Behaviour,
        attr:dict={},
    ):
        name: str = attr['name'] if 'name' in attr.keys() else 'RunningIsFailure'
        super().__init__(name, child)