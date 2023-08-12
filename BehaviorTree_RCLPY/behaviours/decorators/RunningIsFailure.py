from py_trees.behaviour import Behaviour
from py_trees.decorators import RunningIsFailure as pyRunningIsFailure


class RunningIsFailure(pyRunningIsFailure):
    def __init__(self, name: str, child: Behaviour):
        super().__init__(name, child)