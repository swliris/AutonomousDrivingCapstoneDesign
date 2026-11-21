import pytest
from second.config import vehicle_from
from second.demo import fixtures


@pytest.fixture
def setup():
    config, course = fixtures()
    return config, course, vehicle_from(config)
