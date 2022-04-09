import pytest

from modulo_components.component_interface import ComponentInterface


def raise_(ex):
    raise ex


@pytest.fixture()
def component_interface(ros_context):
    yield ComponentInterface('component_interface')


def test_add_bool_predicate(component_interface):
    component_interface.add_predicate('foo', True)
    assert 'foo' in component_interface._predicates.keys()
    assert component_interface._predicates['foo']


def test_add_function_predicate(component_interface):
    component_interface.add_predicate('foo', lambda: False)
    assert 'foo' in component_interface._predicates.keys()
    assert not component_interface._predicates['foo']()


def test_get_predicate(component_interface):
    component_interface.add_predicate('foo', True)
    assert component_interface.get_predicate('foo')
    component_interface.add_predicate('bar', lambda: True)
    assert component_interface.get_predicate('bar')
    # predicate does not exist, expect false
    assert not component_interface.get_predicate('test')
    # error in callback function except false
    component_interface.add_predicate('error', lambda: raise_(RuntimeError("An error occurred")))
    assert not component_interface.get_predicate('error')


def test_set_predicate(component_interface):
    component_interface.add_predicate('foo', True)
    component_interface.set_predicate('foo', False)
    assert not component_interface.get_predicate('foo')
    # predicate does not exist so setting won't do anything
    component_interface.set_predicate('bar', True)
    assert not component_interface.get_predicate('bar')
    component_interface.add_predicate('bar', True)
    component_interface.set_predicate('bar', lambda: False)
    assert not component_interface.get_predicate('bar')
