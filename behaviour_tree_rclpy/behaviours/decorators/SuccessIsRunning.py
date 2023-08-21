from py_trees.behaviour import Behaviour
from py_trees.decorators import SuccessIsRunning as pySuccessIsRunning


class SuccessIsRunning(pySuccessIsRunning):
    def __init__(
        self, 
        child: Behaviour,
        **kwargs,
    ):
        kwargs = kwargs['kwargs']
        name: str = kwargs['name'] if 'name' in kwargs.keys() else 'SuccessIsRunning'
        super().__init__(name, child)