from py_trees.behaviour import Behaviour
from py_trees.decorators import FailureIsRunning as pyFailureIsRunning


class FailureIsRunning(pyFailureIsRunning):
   def __init__(
        self, 
        child: Behaviour,
        attr:dict={},
    ):
        name: str = attr['name'] if 'name' in attr.keys() else 'FailureIsRunning'
        super().__init__(name, child)