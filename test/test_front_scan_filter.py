import math

from drive_manager.front_scan_filter import angle_is_in_sector


def test_wrapped_front_sector_keeps_sensor_pi_direction():
    center = math.pi
    half_angle = math.radians(120.0)

    assert angle_is_in_sector(math.pi, center, half_angle)
    assert angle_is_in_sector(-math.pi, center, half_angle)
    assert angle_is_in_sector(math.radians(61.0), center, half_angle)
    assert angle_is_in_sector(math.radians(-61.0), center, half_angle)


def test_wrapped_front_sector_rejects_robot_rear_direction():
    center = math.pi
    half_angle = math.radians(120.0)

    assert not angle_is_in_sector(0.0, center, half_angle)
    assert not angle_is_in_sector(math.radians(59.0), center, half_angle)
    assert not angle_is_in_sector(math.radians(-59.0), center, half_angle)


def test_sector_includes_exact_boundary():
    center = math.pi
    half_angle = math.radians(120.0)

    assert angle_is_in_sector(math.radians(60.0), center, half_angle)
    assert angle_is_in_sector(math.radians(-60.0), center, half_angle)
