import pytest

from wlsonar._semver import _semver_is_less_than


@pytest.mark.parametrize(
    "a, b",
    [
        ("1.2.3", "1.2.4"),
        ("1.2.3", "1.3.0"),
        ("1.2.3", "2.0.0"),
    ],
)
def test_less_than(a: str, b: str) -> None:
    assert _semver_is_less_than(a, b)


@pytest.mark.parametrize(
    "a, b",
    [
        ("1.2.3", "1.2.3"),
    ],
)
def test_equal(a: str, b: str) -> None:
    assert not _semver_is_less_than(a, b)


@pytest.mark.parametrize(
    "a, b",
    [
        ("1.2.4", "1.2.3"),
        ("1.3.0", "1.2.9"),
        ("2.0.0", "1.9.9"),
    ],
)
def test_greater_than(a: str, b: str) -> None:
    assert not _semver_is_less_than(a, b)


@pytest.mark.parametrize(
    "a, b",
    [
        ("1.2", "1.2.3"),
        ("1.2.3.4", "1.2.3"),
        ("abc", "1.2.3"),
        ("1.2.3", "xyz"),
        ("", "1.2.3"),
        ("1.2.3", ""),
        (0, "1.2.3"),
    ],
)
def test_invalid_input_raises(a: str, b: str) -> None:
    with pytest.raises(Exception):
        _semver_is_less_than(a, b)
