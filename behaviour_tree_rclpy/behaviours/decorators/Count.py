from py_trees.behaviour import Behaviour
from py_trees.decorators import Count as pyCount


class Count(pyCount):
    def __init__(
        self, 
        child: Behaviour,
        **kwargs,
    ):
        kwargs = kwargs['kwargs']
        name: str = kwargs['name'] if 'name' in kwargs.keys() else 'Count'
        super().__init__(name, child)