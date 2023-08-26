from py_trees.behaviour import Behaviour
from py_trees.decorators import FailureIsSuccess as pyFailureIsSuccess


class FailureIsSuccess(pyFailureIsSuccess):
    def __init__(
        self, 
        child: Behaviour,
        attr: dict={},
    ):
        name: str = attr['name'] if 'name' in attr.keys() else 'FailureIsSuccess'
        super().__init__(name, child)