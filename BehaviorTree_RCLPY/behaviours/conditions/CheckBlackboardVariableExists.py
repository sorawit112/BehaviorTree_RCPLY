import typing
from py_trees import common
from py_trees.behaviours import CheckBlackboardVariableExists as pyCheckBlackboardVariableExists

class CheckBlackboardVariableExists(pyCheckBlackboardVariableExists):
    def __init__(self, name: str, variable_name: str):
        if not isinstance(variable_name, str):
            raise TypeError(f"variable_name type:{type(variable_name)} not supported")
    
        super().__init__(name, variable_name)