"""Statistics, static noise estimates and Allan deviation for IMU recordings."""
from dataclasses import dataclass

import numpy as np

GRAVITY = 9.80665
RAD2DEG = 180.0 / np.pi
# sigma_min = 0.664 * B for a flicker-noise bias instability B (IEEE Std 952)
BIAS_INSTABILITY_FACTOR = np.sqrt(2.0 * np.log(2.0) / np.pi)

AXES = ('x', 'y', 'z')


@dataclass
class TimingStats:
    samples: int
    duration: float
    rate: float
    stamp_dt_max: float
    gaps: int  # stamp intervals longer than 1.5 mean periods
    non_monotonic: int
    arrival_dt_max: float = None
    arrival_gaps: int = None


def timing_stats(rec):
    dt = np.diff(rec.stamp)
    mean_dt = rec.dt
    stats = TimingStats(
        samples=len(rec),
        duration=rec.duration,
        rate=1.0 / mean_dt,
        stamp_dt_max=float(dt.max()),
        gaps=int(np.sum(dt > 1.5 * mean_dt)),
        non_monotonic=int(np.sum(dt <= 0)),
    )
    if rec.arrival is not None:
        adt = np.diff(rec.arrival)
        stats.arrival_dt_max = float(adt.max())
        stats.arrival_gaps = int(np.sum(adt > 1.5 * mean_dt))
    return stats


@dataclass
class StaticSummary:
    gyro_mean: np.ndarray  # rad/s, the gyro bias if the sensor was still
    gyro_var: np.ndarray  # rad^2/s^2
    acc_mean: np.ndarray  # m/s^2
    acc_var: np.ndarray  # m^2/s^4
    gyro_drift: np.ndarray  # last third mean - first third mean, rad/s
    acc_drift: np.ndarray
    acc_magnitude: float


def static_summary(rec):
    third = len(rec) // 3
    return StaticSummary(
        gyro_mean=rec.gyro.mean(axis=0),
        gyro_var=rec.gyro.var(axis=0, ddof=1),
        acc_mean=rec.acc.mean(axis=0),
        acc_var=rec.acc.var(axis=0, ddof=1),
        gyro_drift=rec.gyro[-third:].mean(axis=0) - rec.gyro[:third].mean(axis=0),
        acc_drift=rec.acc[-third:].mean(axis=0) - rec.acc[:third].mean(axis=0),
        acc_magnitude=float(np.linalg.norm(rec.acc.mean(axis=0))),
    )


def allan_deviation(rate, dt, points=100):
    """Overlapping Allan deviation of a rate signal.

    `rate` is integrated to an angle (or velocity) and sigma^2(tau) is
    1/(2 tau^2) <(theta[k+2m] - 2 theta[k+m] + theta[k])^2>, with tau = m dt.
    Cluster times go up to a ninth of the recording, beyond which too few
    independent averages are left. Returns (tau, adev, adev_error) where the
    error is the approximate 1-sigma uncertainty of adev.
    """
    theta = np.concatenate(([0.0], np.cumsum(rate) * dt))
    n = len(theta)
    m_max = max(2, (n - 1) // 9)
    ms = np.unique(np.round(np.logspace(0, np.log10(m_max), points)).astype(int))
    adev = np.empty(len(ms))
    for i, m in enumerate(ms):
        second_diff = theta[2 * m:] - 2.0 * theta[m:-m] + theta[:-2 * m]
        adev[i] = np.sqrt(np.mean(second_diff ** 2) / 2.0) / (m * dt)
    error = adev / np.sqrt(2.0 * (n / ms - 1.0))
    return ms * dt, adev, error


@dataclass
class NoiseParameters:
    random_walk: float  # N, signal units / sqrt(Hz): ARW (rad/s/sqrt(Hz)) or VRW ((m/s^2)/sqrt(Hz))
    bias_instability: float  # B, signal units
    bias_instability_tau: float  # s
    bias_instability_resolved: bool
    rate_random_walk: float  # K, signal units / sqrt(s), None when not visible
    sample_period: float


def noise_parameters(tau, adev, dt):
    """Read the standard noise terms off an Allan deviation curve.

    N is the median of adev*sqrt(tau) (slope -1/2) from 10 samples up to a third
    of the minimum's tau, where the sensor's low-pass filter no longer
    correlates the samples. B is the minimum divided by 0.664. K is the median
    of adev*sqrt(3/tau) (slope +1/2) beyond three times the minimum's tau, if
    the curve is rising there.
    """
    i_min = int(np.argmin(adev))
    tau_min = tau[i_min]

    white = (tau >= 10 * dt) & (tau <= tau_min / 3.0)
    if not white.any():
        white = (tau >= 4 * dt) & (tau <= tau_min)
    if not white.any():
        white = tau <= tau_min
    random_walk = float(np.median(adev[white] * np.sqrt(tau[white])))

    rate_random_walk = None
    walk = tau >= 3.0 * tau_min
    if walk.sum() >= 3:
        slope = np.polyfit(np.log(tau[walk]), np.log(adev[walk]), 1)[0]
        if slope > 0.25:
            rate_random_walk = float(np.median(adev[walk] * np.sqrt(3.0 / tau[walk])))

    return NoiseParameters(
        random_walk=random_walk,
        bias_instability=float(adev[i_min] / BIAS_INSTABILITY_FACTOR),
        bias_instability_tau=float(tau_min),
        bias_instability_resolved=bool(tau_min < 0.5 * tau[-1]),
        rate_random_walk=rate_random_walk,
        sample_period=dt,
    )


def gyro_report_units(p):
    """ARW in deg/sqrt(h), bias instability in deg/h, RRW in deg/h/sqrt(h)."""
    return {
        'ARW [deg/sqrt(h)]': p.random_walk * RAD2DEG * 60.0,
        'bias instability [deg/h]': p.bias_instability * RAD2DEG * 3600.0,
        'RRW [deg/h/sqrt(h)]': None if p.rate_random_walk is None
        else p.rate_random_walk * RAD2DEG * 3600.0 * 60.0,
    }


def acc_report_units(p):
    """VRW in m/s/sqrt(h), bias instability in mg, RRW in mg/sqrt(h)."""
    return {
        'VRW [m/s/sqrt(h)]': p.random_walk * 60.0,
        'bias instability [mg]': p.bias_instability / GRAVITY * 1e3,
        'RRW [mg/sqrt(h)]': None if p.rate_random_walk is None
        else p.rate_random_walk / GRAVITY * 1e3 * 60.0,
    }
