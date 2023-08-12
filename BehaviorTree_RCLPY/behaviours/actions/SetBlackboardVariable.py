import typing
from py_trees import common
from py_trees.behaviours import SetBlackboardVariable as pySetBlackboardVariable

class SetBlackboardVariable(pySetBlackboardVariable):
    def __init__(
        self, 
        name: str, 
        variable_name: str, 
        variable_value: typing.Any, 
        overwrite: typing.Union[str, bool]
    ):
        if isinstance(variable_value, str):
            if variable_value.startswith("{") and variable_value.endswith("}"):
                # key = variable_value[1:-1]
                # self.blackboard = self.attach_blackboard_client()
                # self.blackboard.register_key(key=key, access=common.Access.READ)        
                # try:
                #     variable_value = self.blackboard.get(key)
                # except AttributeError as e:
                #     raise AttributeError(e)
                # except KeyError as e:
                #     raise KeyError(e)
                raise ValueError(f"input string {variable_value} not supported remapping key")
            else:
                try:
                    variable_value = float(variable_value)
                except ValueError:
                    if variable_value.lower() in ['true', 'false']:
                        variable_value = bool(variable_value)
                    else:
                        raise ValueError(f"input string {variable_value} can't convert to type 'Float' or 'Bool'")
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