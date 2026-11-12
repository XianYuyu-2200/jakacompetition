import pytest

from jaka_competition_kit.reference import object_height


def test_object_height_uses_legacy_size_field():
    assert object_height({"id": "wp_1", "size": 0.05}) == 0.05


def test_object_height_falls_back_to_dimensions_for_manifest_without_size():
    assert object_height({"id": "wp_1", "dims": [0.04, 0.06, 0.03]}) == 0.03


def test_object_height_reports_invalid_manifest():
    with pytest.raises(ValueError, match="缺少 size"):
        object_height({"id": "wp_1", "dims": [0.04, 0.06]})
