from py_trees.behaviour import Behaviour
from py_trees.decorators import PassThrough as pyPassThrough


class PassThrough(pyPassThrough):
    def __init__(
        self, 
        child: Behaviour,
        **kwargs,
    ):
        kwargs = kwargs['kwargs']
        name: str = kwargs['name'] if 'name' in kwargs.keys() else 'PassThrough'
        super().__init__(name, child)