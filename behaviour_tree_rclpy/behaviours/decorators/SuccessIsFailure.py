from py_trees.behaviour import Behaviour
from py_trees.decorators import SuccessIsFailure as pySuccessIsFailure


class SuccessIsFailure(pySuccessIsFailure):
    def __init__(
        self, 
        child: Behaviour,
        attr:dict={},
    ):
        name: str = attr['name'] if 'name' in attr.keys() else 'SuccessIsFailure'
        super().__init__(name, child)