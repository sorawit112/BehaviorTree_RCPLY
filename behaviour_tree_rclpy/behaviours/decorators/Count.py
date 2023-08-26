from py_trees.behaviour import Behaviour
from py_trees.decorators import Count as pyCount


class Count(pyCount):
    def __init__(
        self, 
        child: Behaviour,
        attr:dict={},
    ):
        name: str = attr['name'] if 'name' in attr.keys() else 'Count'
        super().__init__(name, child)