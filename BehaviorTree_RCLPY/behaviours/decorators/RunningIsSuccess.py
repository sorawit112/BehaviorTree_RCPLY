from py_trees.behaviour import Behaviour
from py_trees.decorators import RunningIsSuccess as pyRunningIsSuccess


class RunningIsSuccess(pyRunningIsSuccess):
    def __init__(self, name: str, child: Behaviour):
        super().__init__(name, child)