import typing
from py_trees.behaviour import Behaviour
from py_trees.composites import Selector

class Fallback(Selector):
    def __init__(
        self, 
        name: str, 
        children: typing.Optional[typing.List[Behaviour]] = None):
        super().__init__(name, False, children)