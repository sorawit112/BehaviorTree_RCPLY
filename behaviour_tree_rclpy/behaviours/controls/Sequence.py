import typing
from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence as pySequence

class Sequence(pySequence):
    def __init__(
        self, 
        children: typing.Optional[typing.List[Behaviour]],
        **kwargs,
    ):
        kwargs = kwargs['kwargs']
        name: str = kwargs['name'] if 'name' in kwargs.keys() else 'Sequence'
        super().__init__(name, False, children)