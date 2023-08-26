import typing
from py_trees.behaviour import Behaviour
from py_trees.composites import Selector

class Fallback(Selector):
    def __init__(
        self, 
        children: typing.Optional[typing.List[Behaviour]],
        attr:dict={},
    ):
        name: str = attr['name'] if 'name' in attr.keys() else 'Fallback'
        super().__init__(name, False, children)