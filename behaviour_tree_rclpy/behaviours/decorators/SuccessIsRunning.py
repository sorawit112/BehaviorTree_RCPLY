from py_trees.behaviour import Behaviour
from py_trees.decorators import SuccessIsRunning as pySuccessIsRunning


class SuccessIsRunning(pySuccessIsRunning):
    def __init__(
        self, 
        child: Behaviour,
        attr:dict={},
    ):
        name: str = attr['name'] if 'name' in attr.keys() else 'SuccessIsRunning'
        super().__init__(name, child)