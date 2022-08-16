class CoreError(Exception):
    """
    A base class for all core exceptions.
    """

    def __init__(self, message: str, prefix="CoreError"):
        super().__init__(f"{prefix}: {message}")


class MessageTranslationError(CoreError):
    """
    An exception class to notify that the translation of a ROS message failed.
    """

    def __init__(self, message: str):
        super().__init__(message, "MessageTranslationError")


class ParameterTranslationError(CoreError):
    """
    An exception class to notify incompatibility when translating parameters from different sources. This is an
    exception class to be thrown if there is a problem while translating from a ROS parameter to a state_representation
    parameter and vice versa.
    """

    def __init__(self, message: str):
        super().__init__(message, "ParameterTranslationError")
