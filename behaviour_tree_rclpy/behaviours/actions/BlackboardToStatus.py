from py_trees.behaviours import BlackboardToStatus as pyBlackboardToStatus

class BlackboardToStatus(pyBlackboardToStatus):
    def __init__(self, attr:dict={}):
        name: str = attr['name'] if 'name' in attr.keys() else 'BlackboardToStatus'
        variable_name: str = attr['variable_name']
        super().__init__(name, variable_name)