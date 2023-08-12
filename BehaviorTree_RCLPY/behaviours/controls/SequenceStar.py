import typing
from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence as pySequence

class SequenceStar(pySequence):
    def __init__(
        self, 
        name: str, 
        children: typing.Optional[typing.List[Behaviour]] = None):
        super().__init__(name, True, children)