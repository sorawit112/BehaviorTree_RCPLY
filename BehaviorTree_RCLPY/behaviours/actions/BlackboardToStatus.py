from py_trees.behaviours import BlackboardToStatus as pyBlackboardToStatus

class BlackboardToStatus(pyBlackboardToStatus):
    def __init__(self, name: str, variable_name: str):
        super().__init__(name, variable_name)