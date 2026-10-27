"""Check the analysis against synthetic data with known noise parameters."""
import numpy as np
import pytest

from stim300_tools import analysis, recording

RATE = 250.0
DT = 1.0 / RATE


def make_recording(gyro, acc):
    stamp = np.arange(len(gyro)) * DT
    return recording.Recording(stamp, gyro, acc)


def test_timing_stats_counts_gaps():
    stamp = np.arange(1000) * DT
    stamp[500:] += 0.05
    rec = recording.Recording(stamp, np.zeros((1000, 3)), np.zeros((1000, 3)))
    stats = analysis.timing_stats(rec)
    assert stats.gaps == 1
    assert stats.non_monotonic == 0
    assert stats.stamp_dt_max == pytest.approx(DT + 0.05)


def test_static_summary_recovers_bias_and_variance():
    rng = np.random.default_rng(1)
    n = 100000
    gyro = 0.001 + 0.002 * rng.standard_normal((n, 3))
    acc = np.array([0.1, -0.2, 9.8]) + 0.01 * rng.standard_normal((n, 3))
    s = analysis.static_summary(make_recording(gyro, acc))
    assert s.gyro_mean == pytest.approx([0.001] * 3, abs=5e-5)
    assert s.gyro_var == pytest.approx([4e-6] * 3, rel=0.03)
    assert s.acc_var == pytest.approx([1e-4] * 3, rel=0.03)
    assert s.acc_magnitude == pytest.approx(np.linalg.norm([0.1, -0.2, 9.8]), abs=1e-3)


def test_allan_deviation_of_white_noise_has_slope_minus_half():
    rng = np.random.default_rng(2)
    sigma = 0.01
    rate = sigma * rng.standard_normal(500000)
    tau, adev, _ = analysis.allan_deviation(rate, DT)
    # white noise of std sigma sampled at DT: N = sigma * sqrt(DT)
    expected = sigma * np.sqrt(DT) / np.sqrt(tau)
    assert adev == pytest.approx(expected, rel=0.15)


def test_noise_parameters_recover_random_walk_and_rate_random_walk():
    rng = np.random.default_rng(1)
    n = 1500000
    n_white = 1e-3  # signal units / sqrt(Hz)
    k_walk = 2e-4  # signal units / sqrt(s)
    white = n_white / np.sqrt(DT) * rng.standard_normal(n)
    walk = np.cumsum(k_walk * np.sqrt(DT) * rng.standard_normal(n))
    tau, adev, _ = analysis.allan_deviation(white + walk, DT)
    p = analysis.noise_parameters(tau, adev, DT)
    assert p.random_walk == pytest.approx(n_white, rel=0.1)
    assert p.rate_random_walk == pytest.approx(k_walk, rel=0.2)
    assert p.bias_instability_resolved


def test_pure_white_noise_has_no_rate_random_walk():
    rng = np.random.default_rng(4)
    rate = 0.01 * rng.standard_normal(500000)
    tau, adev, _ = analysis.allan_deviation(rate, DT)
    assert analysis.noise_parameters(tau, adev, DT).rate_random_walk is None


def test_recording_roundtrip(tmp_path):
    rng = np.random.default_rng(5)
    rec = make_recording(rng.standard_normal((50, 3)), rng.standard_normal((50, 3)))
    path = tmp_path / 'rec.npz'
    rec.save(path)
    loaded = recording.load(str(path), '/unused')
    assert np.array_equal(loaded.gyro, rec.gyro)
    assert np.array_equal(loaded.stamp, rec.stamp)
