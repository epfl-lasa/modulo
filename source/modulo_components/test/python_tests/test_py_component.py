import pytest

from template_component_package.py_component import PYComponent


@pytest.fixture()
def py_component(ros_context):
    yield PYComponent('py_component')


def test_construction(py_component):
    assert py_component.get_name() == 'py_component'
