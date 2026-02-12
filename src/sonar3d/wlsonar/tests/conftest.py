import pytest


def pytest_addoption(parser: pytest.Parser) -> None:
    parser.addoption("--sonar-ip", action="store", default=None, help="Sonar IP address")
