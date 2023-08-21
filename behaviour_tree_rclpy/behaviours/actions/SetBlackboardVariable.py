import typing
from py_trees import common
from py_trees.behaviours import SetBlackboardVariable as pySetBlackboardVariable

class SetBlackboardVariable(pySetBlackboardVariable):
    def __init__(self, **kwargs):
        kwargs = kwargs['kwargs']
        name: str = kwargs['name'] if 'name' in kwargs.keys() else 'SetBlackBoardVariable'
        variable_name: str = kwargs['variable_name']
        variable_value: typing.Any = kwargs['variable_value']
        overwrite: typing.Union[str, bool] = kwargs['overwrite']
        
        if isinstance(variable_value, str):
            if variable_value.startswith("{") and variable_value.endswith("}"):
                raise ValueError(f"input string {variable_value} not supported remapping key")
            else:
                try:
                    variable_value = float(variable_value)
                except ValueError:
                    if variable_value.lower() in ['true', 'false']:
                        variable_value = bool(variable_value)
                    else:
                        print('variable_value in setBlackBoard is type string')
        elif not isinstance(variable_value, (float, int, bool)):
            raise TypeError(f"variable_value type:{type(variable_value)} not supported")
        
        if isinstance(overwrite, str):
            if overwrite.lower() in ['true', 'false']:
                overwrite = bool(overwrite)
            else:
                raise ValueError(f"input string {overwrite} can't convert to Bool")
        elif not isinstance(overwrite, bool):
            raise TypeError(f"variable_value type:{type(variable_value)} not supported")
        
        super().__init__(name, variable_name, variable_value, overwrite)