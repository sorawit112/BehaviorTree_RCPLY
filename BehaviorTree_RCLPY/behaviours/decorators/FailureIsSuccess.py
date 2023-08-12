from py_trees.behaviour import Behaviour
from py_trees.decorators import FailureIsSuccess as pyFailureIsSuccess


class FailureIsSuccess(pyFailureIsSuccess):
    def __init__(self, name: str, child: Behaviour):
        super().__init__(name, child)