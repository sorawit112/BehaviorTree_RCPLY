from py_trees.behaviour import Behaviour
from py_trees.decorators import FailureIsRunning as pyFailureIsRunning


class FailureIsRunning(pyFailureIsRunning):
   def __init__(
        self, 
        child: Behaviour,
        **kwargs,
    ):
        kwargs = kwargs['kwargs']
        name: str = kwargs['name'] if 'name' in kwargs.keys() else 'FailureIsRunning'
        super().__init__(name, child)