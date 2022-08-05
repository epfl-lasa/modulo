# Modulo Component Interfaces

This package defines custom standard interfaces for modulo components.

## Services

Modulo component classes provide a simplified method to add services which trigger a pre-defined callback function.
Services are either empty (with no request payload) or carry a string request. They return a common
modulo_components::ComponentServiceResponse structure with a success flag and status message.

### EmptyTrigger

The EmptyTrigger service request takes no parameters.
The response contains a boolean `success` flag and a string `message`.

### StringTrigger

The EmptyTrigger service request takes a string `payload`.
The response contains a boolean `success` flag and a string `message`.
It is the responsibility of the component to define the expected payload format and to document it appropriately.