import typing
from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence as pySequence

class SequenceStar(pySequence):
    def __init__(
        self, 
        children: typing.Optional[typing.List[Behaviour]],
        **kwargs,
    ):
        kwargs = kwargs['kwargs']
        name: str = kwargs['name'] if 'name' in kwargs.keys() else 'SequenceStar'
        super().__init__(name, True, children)