from py_trees.behaviour import Behaviour
from py_trees.decorators import RunningIsSuccess as pyRunningIsSuccess


class RunningIsSuccess(pyRunningIsSuccess):
    def __init__(
        self, 
        child: Behaviour,
        **kwargs,
    ):
        kwargs = kwargs['kwargs']
        name: str = kwargs['name'] if 'name' in kwargs.keys() else 'RunningIsSuccess'
        super().__init__(name, child)