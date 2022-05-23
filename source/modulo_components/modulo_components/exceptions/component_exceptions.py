class ComponentError(Exception):
    def __init__(self, message):
        super().__init__(message)


class ComponentParameterError(ComponentError):
    def __init__(self, message):
        super().__init__(message)
