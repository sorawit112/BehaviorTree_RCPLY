import typing
from .SetBlackboardVariable import SetBlackboardVariable

class SetBlackboard(SetBlackboardVariable):
    def __init__(self, **kwargs):
        kwargs = kwargs['kwargs']
        name: str = kwargs['name'] if 'name' in kwargs.keys() else 'SetBlackboard'
        variable_name: str = kwargs['output_key']
        variable_value: typing.Any = kwargs['value']
        
        kwargs['name'] = name
        kwargs['variable_name'] = variable_name
        kwargs['variable_value'] = variable_value
        kwargs['overwrite'] = True
        
        super().__init__(kwargs=kwargs)