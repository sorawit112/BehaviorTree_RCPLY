from py_trees.behaviour import Behaviour
from py_trees.decorators import Count as pyCount


class Count(pyCount):
    def __init__(self, name: str, child: Behaviour):
        super().__init__(name, child)