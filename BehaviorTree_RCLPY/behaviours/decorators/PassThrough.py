from py_trees.behaviour import Behaviour
from py_trees.decorators import PassThrough as pyPassThrough


class PassThrough(pyPassThrough):
    def __init__(self, name: str, child: Behaviour):
        super().__init__(name, child)