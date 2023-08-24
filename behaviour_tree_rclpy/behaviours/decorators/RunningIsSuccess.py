from py_trees.behaviour import Behaviour
from py_trees.decorators import RunningIsSuccess as pyRunningIsSuccess


class RunningIsSuccess(pyRunningIsSuccess):
    def __init__(
        self, 
        child: Behaviour,
        attr:dict={},
    ):
        name: str = attr['name'] if 'name' in attr.keys() else 'RunningIsSuccess'
        super().__init__(name, child)