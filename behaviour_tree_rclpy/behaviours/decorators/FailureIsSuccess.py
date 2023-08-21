from py_trees.behaviour import Behaviour
from py_trees.decorators import FailureIsSuccess as pyFailureIsSuccess


class FailureIsSuccess(pyFailureIsSuccess):
    def __init__(
        self, 
        child: Behaviour,
        **kwargs,
    ):
        kwargs = kwargs['kwargs']
        name: str = kwargs['name'] if 'name' in kwargs.keys() else 'FailureIsSuccess'
        super().__init__(name, child)