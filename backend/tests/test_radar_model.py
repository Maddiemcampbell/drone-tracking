import math

import numpy as np
import pytest

from app.tracking.radar import RadarMeasurementModel


def test_radar_model_returns_range_and_sensor_relative_bearing():
    model = RadarMeasurementModel(sensor_position=(1, 2), sensor_heading_degrees=30)
    measurement = model.predict([1 + math.sqrt(3), 3, 8, -4])
    assert measurement == pytest.approx([2, 0])


def test_radar_model_normalizes_bearing_across_pi_boundary():
    model = RadarMeasurementModel(sensor_heading_degrees=179)
    measurement = model.predict([math.cos(math.radians(-179)), math.sin(math.radians(-179)), 0, 0])
    assert measurement[1] == pytest.approx(math.radians(2))


def test_radar_model_jacobian_matches_position_finite_difference():
    model = RadarMeasurementModel(sensor_position=(-2, 3), sensor_heading_degrees=25)
    state = np.array([8.0, -4.0, 5.0, -2.0])
    epsilon = 1e-6
    columns = []
    for index in range(4):
        delta = model.predict(state + np.eye(4)[index] * epsilon) - model.predict(state - np.eye(4)[index] * epsilon)
        delta[1] = (delta[1] + math.pi) % (2 * math.pi) - math.pi
        columns.append(delta / (2 * epsilon))
    numerical = np.column_stack(columns)
    np.testing.assert_allclose(model.jacobian(state), numerical, atol=1e-6)


def test_radar_model_jacobian_has_no_direct_velocity_sensitivity():
    model = RadarMeasurementModel()
    jacobian = model.jacobian([3, 4, 20, -10])
    np.testing.assert_allclose(jacobian[:, 2:], 0)


def test_radar_model_covariance_uses_meters_squared_and_radians_squared():
    covariance = RadarMeasurementModel.covariance(4, 3)
    np.testing.assert_allclose(covariance, [[16, 0], [0, math.radians(3) ** 2]])
    np.testing.assert_allclose(RadarMeasurementModel.covariance(0, 0), np.zeros((2, 2)))


def test_radar_model_rejects_origin_and_negative_noise():
    with pytest.raises(ValueError, match="origin"):
        RadarMeasurementModel().predict([0, 0, 0, 0])
    with pytest.raises(ValueError, match="origin"):
        RadarMeasurementModel().jacobian([0, 0, 0, 0])
    with pytest.raises(ValueError, match="range noise"):
        RadarMeasurementModel.covariance(-1, 2)
    with pytest.raises(ValueError, match="bearing noise"):
        RadarMeasurementModel.covariance(1, -2)
