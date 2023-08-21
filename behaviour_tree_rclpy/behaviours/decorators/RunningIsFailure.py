from py_trees.behaviour import Behaviour
from py_trees.decorators import RunningIsFailure as pyRunningIsFailure


class RunningIsFailure(pyRunningIsFailure):
    def __init__(
        self, 
        child: Behaviour,
        **kwargs,
    ):
        kwargs = kwargs['kwargs']
        name: str = kwargs['name'] if 'name' in kwargs.keys() else 'RunningIsFailure'
        super().__init__(name, child)