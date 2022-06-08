# Contributing

This document is still work in progress.

### Exceptions

This section describes how a component should handle exceptions and errors.

As a general rule, a component interface methods do not throw an exception or raise an error unless there is no other
meaningful option. More precisely, only non-void public/protected methods throw, i.e. all setters and `add_xxx` methods 
do not throw but catch all exceptions and log an error.

If an exception is thrown, it is either a `ComponentException` (in C++) or a `ComponentError` (in Python) or any
derived exception, such that all exceptions thrown by a component can be caught with those base exceptions (for example
in the periodic `step` function).

Currently, the following methods throw:
- `get_parameter`
- `get_parameter_value`
- `lookup_transform`

### Logging

Similar to the exceptions, the logging of debug, info, and error messages should follow some principles:

- Methods that catch an exception and are not allow to rethrow, log an error with the exception message
- `add_xxx` methods use non-throttled logging
- Setters and getters as well as all other methods that are expected to be called at a high frequency use throttled
  logging