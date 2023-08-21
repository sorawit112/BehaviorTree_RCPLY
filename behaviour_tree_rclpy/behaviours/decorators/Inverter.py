from py_trees.behaviour import Behaviour
from py_trees.decorators import Inverter as pyInverter


class Inverter(pyInverter):
    def __init__(
        self, 
        child: Behaviour,
        **kwargs,
    ):
        kwargs = kwargs['kwargs']
        name: str = kwargs['name'] if 'name' in kwargs.keys() else 'Inverter'
        super().__init__(name, child)