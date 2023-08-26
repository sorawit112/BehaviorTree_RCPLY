import typing
from .SetBlackboardVariable import SetBlackboardVariable

class SetBlackboard(SetBlackboardVariable):
    def __init__(self, attr:dict={}):
        name: str = attr['name'] if 'name' in attr.keys() else 'SetBlackboard'
        variable_name: str = attr['output_key']
        variable_value: typing.Any = attr['value']
        
        attr['name'] = name
        attr['variable_name'] = variable_name
        attr['variable_value'] = variable_value
        attr['overwrite'] = True
        
        super().__init__(attr=attr)