from py_trees.behaviour import Behaviour
from py_trees.decorators import SuccessIsFailure as pySuccessIsFailure


class SuccessIsFailure(pySuccessIsFailure):
    def __init__(
        self, 
        child: Behaviour,
        **kwargs,
    ):
        kwargs = kwargs['kwargs']
        name: str = kwargs['name'] if 'name' in kwargs.keys() else 'SuccessIsFailure'
        super().__init__(name, child)