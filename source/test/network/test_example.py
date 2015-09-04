import pytest
from pytest import raises


def multiply(a, b):
    return a * b


def test_numbers_3_4():
    assert multiply(3, 4) == 12


def test_strings_a_3():
    assert multiply('a', 3) == 'aaa'