from py_trees.behaviour import Behaviour
from py_trees.decorators import Inverter as pyInverter


class Inverter(pyInverter):
    def __init__(
        self, 
        child: Behaviour,
        attr: dict={},
    ):
        name: str = attr['name'] if 'name' in attr.keys() else 'Inverter'
        super().__init__(name, child)