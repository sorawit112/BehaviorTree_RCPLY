import typing
import operator
from py_trees import common
from py_trees.behaviours import CheckBlackboardVariableValue as pyCheckBlackboardVariableValue

operator_mapping = {
    '=' : operator.eq, '!=' : operator.ne,
    '>' : operator.gt, '>=' : operator.ge,
    '<' : operator.lt, '<=' : operator.le,
}

class CheckBlackboardVariableValue(pyCheckBlackboardVariableValue):
    def __init__(self, **kwargs):
        kwargs = kwargs['kwargs']
        name: str = kwargs['name'] if 'name' in kwargs.keys() else 'CheckBlackboardVariableValue'
        operator: typing.Union[str, common.ComparisonExpression] = kwargs['operator']
        value: typing.Any = kwargs['value']
        variable_name: str = kwargs['variable_name']
        if not isinstance(variable_name, str):
            raise TypeError(f"variable_name type:{type(variable_name)} not supported")
        
        if isinstance(value, str):
            if value.startswith("{") and value.endswith("}"):
                raise ValueError(f"input string {value} not supported remapping key")
            else:
                try:
                    value = float(value)
                except ValueError:
                    if value.lower() in ['true', 'false']:
                        value = bool(value)
                    else:
                        raise ValueError(f"input string {value} can't convert to type 'Float' or 'Bool'")
        
        if isinstance(operator, str):
            try:
                operator = operator_mapping[operator.replace(' ', '')]
                check = common.ComparisonExpression(
                    variable=variable_name,
                    value=value,
                    operator=operator
                )                
            except KeyError as e:
                raise KeyError(f"{e} can't remapping key:{operator} to {list(operator_mapping.values())}")
        super().__init__(name, check)