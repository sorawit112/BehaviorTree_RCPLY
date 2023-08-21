from py_trees.behaviour import Behaviour
from py_trees.decorators import StatusToBlackboard as pyStatusToBlackboard

class StatusToBlackboard(pyStatusToBlackboard):
    def __init__(
        self, 
        child: Behaviour,
        **kwargs,
    ):
        kwargs = kwargs['kwargs']
        name: str = kwargs['name'] if 'name' in kwargs.keys() else 'RunningIsSuccess'
        variable_name: str = kwargs['variable_name']
        super().__init__(name, child, variable_name)

