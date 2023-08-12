from py_trees.behaviour import Behaviour
from py_trees.decorators import Inverter as pyInverter


class Inverter(pyInverter):
    def __init__(self, name: str, child: Behaviour):
        super().__init__(name, child)