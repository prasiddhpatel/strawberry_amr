"""
Static wiring sanity check -- NOT a behavioural test (this sandbox has no
rclpy installed to actually instantiate and drive the real node; that
verification has to happen on the robot/dev machine's real ROS 2
environment). What this test CAN and does verify, without needing rclpy at
all: that base_controller_node.py actually imports AND calls
clamp_command(), rather than merely having command_safety.py exist as an
unused module.

This is not a hypothetical concern -- it is exactly what this workspace's
own review process found: command_safety.py was written, and fully unit-
tested (see test_command_safety.py, 15/15 passing), before anyone noticed
it was never actually imported into base_controller_node.py at all. The
pure-function tests could not have caught that, by design -- they only
ever exercise command_safety.py directly. This test exists specifically to
close that gap: a module can be perfectly correct and perfectly tested and
still not be doing anything for the running robot if nothing calls it.
"""
import ast
import os


def _load_base_controller_ast():
    path = os.path.join(
        os.path.dirname(__file__), '..', 'base_controller', 'base_controller_node.py')
    with open(path) as f:
        return ast.parse(f.read(), filename=path)


class TestClampCommandIsWiredIn:
    def test_command_safety_is_imported(self):
        """base_controller_node.py must import clamp_command from
        command_safety -- not just have command_safety.py exist unused."""
        tree = _load_base_controller_ast()
        imported_names = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom):
                for alias in node.names:
                    imported_names.add(alias.name)
        assert 'clamp_command' in imported_names, (
            "base_controller_node.py does not import clamp_command from "
            "command_safety -- acceleration limiting exists as tested, "
            "unused code, not as something the real robot benefits from.")

    def test_clamp_command_is_actually_called(self):
        """Importing it is necessary but not sufficient -- it must also be
        called somewhere, not just imported and left unused."""
        tree = _load_base_controller_ast()
        call_names = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
                call_names.add(node.func.id)
        assert 'clamp_command' in call_names, (
            "clamp_command is imported but never actually called anywhere "
            "in base_controller_node.py.")
