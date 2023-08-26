import typing
from py_trees.behaviour import Behaviour
from py_trees.composites import Selector

class FallbackStar(Selector):
    def __init__(
        self, 
        children: typing.Optional[typing.List[Behaviour]],
        attr: dict={},
    ):
        name: str = attr['name'] if 'name' in attr.keys() else 'FallbackStar'
        super().__init__(name, True, children)