import typing
from py_trees.behaviour import Behaviour
from py_trees.composites import Selector

class FallbackStar(Selector):
    def __init__(
        self, 
        children: typing.Optional[typing.List[Behaviour]],
        **kwargs,
    ):
        kwargs = kwargs['kwargs']
        name: str = kwargs['ID']
        super().__init__(name, True, children)