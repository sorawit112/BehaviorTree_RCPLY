from py_trees.behaviour import Behaviour
from py_trees.decorators import SuccessIsRunning as pySuccessIsRunning


class SuccessIsRunning(pySuccessIsRunning):
    def __init__(self, name: str, child: Behaviour):
        super().__init__(name, child)