from py_trees.behaviour import Behaviour
from py_trees.decorators import SuccessIsFailure as pySuccessIsFailure


class SuccessIsFailure(pySuccessIsFailure):
    def __init__(self, name: str, child: Behaviour):
        super().__init__(name, child)