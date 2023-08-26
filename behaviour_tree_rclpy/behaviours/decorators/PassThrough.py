from py_trees.behaviour import Behaviour
from py_trees.decorators import PassThrough as pyPassThrough


class PassThrough(pyPassThrough):
    def __init__(
        self, 
        child: Behaviour,
        attr:dict={},
    ):
        name: str = attr['name'] if 'name' in attr.keys() else 'PassThrough'
        super().__init__(name, child)