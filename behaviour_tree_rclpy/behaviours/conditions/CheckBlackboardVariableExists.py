from py_trees.behaviours import CheckBlackboardVariableExists as pyCheckBlackboardVariableExists

class CheckBlackboardVariableExists(pyCheckBlackboardVariableExists):
    def __init__(self, **kwargs):
        kwargs = kwargs['kwargs']
        name: str = kwargs['name'] if 'name' in kwargs.keys() else 'CheckBlackboardVariableExists'
        variable_name: str = kwargs['variable_name']
        if not isinstance(variable_name, str):
            raise TypeError(f"variable_name type:{type(variable_name)} not supported")
    
        super().__init__(name, variable_name)