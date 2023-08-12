import typing
from . import SetBlackboardVariable

class SetBlackboard(SetBlackboardVariable):
    def __init__(
        self, 
        name: str, 
        variable_name: str, 
        variable_value: typing.Any, 
    ):
        super().__init__(name, variable_name, variable_value, True)