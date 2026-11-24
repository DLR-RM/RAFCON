import pytest

import rafcon.core.script
from rafcon.core.config import global_config
from rafcon.core.script import Script

SCRIPT_TEXT = """
calls = 0

def execute(self, inputs, outputs, gvm):
    global calls
    calls += 1
    outputs['calls'] = calls
    return 0
"""


@pytest.fixture
def compile_counter(monkeypatch):
    counter = {'compile': 0}

    def counting_compile(*args, **kwargs):
        counter['compile'] += 1
        return compile(*args, **kwargs)

    monkeypatch.setattr(rafcon.core.script, "compile", counting_compile, raising=False)
    return counter


@pytest.fixture
def recompilation_on():
    old_value = global_config.get_config_value("SCRIPT_RECOMPILATION_ON_STATE_EXECUTION", True)
    global_config.set_config_value("SCRIPT_RECOMPILATION_ON_STATE_EXECUTION", True)
    yield
    global_config.set_config_value("SCRIPT_RECOMPILATION_ON_STATE_EXECUTION", old_value)


def run(script):
    outputs = {'calls': None}
    script.execute(None, {}, outputs)
    return outputs['calls']


def test_unchanged_script_is_compiled_once(compile_counter, recompilation_on):
    script = Script()
    script.script = SCRIPT_TEXT
    # module globals are reset on every execution, as before
    assert run(script) == 1
    assert run(script) == 1
    assert compile_counter['compile'] == 1


def test_changed_script_is_recompiled(compile_counter, recompilation_on):
    script = Script()
    script.script = SCRIPT_TEXT
    run(script)
    script.script = SCRIPT_TEXT.replace("calls += 1", "calls += 2")
    assert run(script) == 2
    assert compile_counter['compile'] == 2


def test_module_kept_without_recompilation(compile_counter):
    old_value = global_config.get_config_value("SCRIPT_RECOMPILATION_ON_STATE_EXECUTION", True)
    global_config.set_config_value("SCRIPT_RECOMPILATION_ON_STATE_EXECUTION", False)
    try:
        script = Script()
        script.script = SCRIPT_TEXT
        assert run(script) == 1
        assert run(script) == 2
        assert compile_counter['compile'] == 1
    finally:
        global_config.set_config_value("SCRIPT_RECOMPILATION_ON_STATE_EXECUTION", old_value)
