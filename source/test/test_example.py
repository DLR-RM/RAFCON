import pytest
import awesome_tool.statemachine.config

def test_basic_math():
    assert 2 + 2 == 4  

if __name__ == '__main__':
    # set the test_libraries path temporarily to the correct value
    library_paths = awesome_tool.statemachine.config.global_config.get_config_value("LIBRARY_PATHS")
    library_paths["test_libraries"] = "../test_scripts/test_libraries"
    pytest.main()
    library_paths = awesome_tool.statemachine.config.global_config.get_config_value("LIBRARY_PATHS")
    library_paths["test_libraries"] = "../../test_scripts/test_libraries"
    awesome_tool.statemachine.config.global_config.save_configuration()