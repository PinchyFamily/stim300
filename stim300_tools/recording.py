"""Get sensor_msgs/Imu samples into numpy arrays, from a running topic, a rosbag2 or a .npz file."""
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class Recording:
    """Samples in SI units. `stamp` is header.stamp in seconds, `arrival` the local receive time."""
    stamp: np.ndarray
    gyro: np.ndarray  # N x 3, rad/s
    acc: np.ndarray  # N x 3, m/s^2
    arrival: Optional[np.ndarray] = None

    def __len__(self):
        return len(self.stamp)

    @property
    def duration(self):
        return float(self.stamp[-1] - self.stamp[0])

    @property
    def dt(self):
        """Mean sample period, which absorbs the occasional dropped sample."""
        return self.duration / (len(self) - 1)

    def save(self, path):
        arrays = {'stamp': self.stamp, 'gyro': self.gyro, 'acc': self.acc}
        if self.arrival is not None:
            arrays['arrival'] = self.arrival
        np.savez_compressed(path, **arrays)

    @staticmethod
    def load(path):
        data = np.load(path)
        return Recording(data['stamp'], data['gyro'], data['acc'],
                         data['arrival'] if 'arrival' in data else None)


def _from_rows(rows, arrival=None):
    data = np.asarray(rows, dtype=np.float64)
    if len(data) < 2:
        raise RuntimeError('Fewer than 2 IMU samples received')
    return Recording(data[:, 0], data[:, 1:4], data[:, 4:7], arrival)


def _row(msg):
    stamp = msg.header.stamp
    w, a = msg.angular_velocity, msg.linear_acceleration
    return (stamp.sec + stamp.nanosec * 1e-9, w.x, w.y, w.z, a.x, a.y, a.z)


def record_topic(topic, duration, progress=None):
    """Subscribe to `topic` for `duration` seconds. `progress(elapsed, n_samples)` is called about once a second."""
    import rclpy
    from sensor_msgs.msg import Imu

    owns_context = not rclpy.ok()
    if owns_context:
        rclpy.init()
    node = rclpy.create_node('imu_recorder')
    rows, arrival = [], []

    def on_imu(msg):
        arrival.append(time.monotonic())
        rows.append(_row(msg))

    node.create_subscription(Imu, topic, on_imu, 10000)
    start = time.monotonic()
    last_report = start
    try:
        while time.monotonic() - start < duration:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if progress and now - last_report >= 1.0:
                progress(now - start, len(rows))
                last_report = now
    finally:
        node.destroy_node()
        if owns_context:
            rclpy.shutdown()
    return _from_rows(rows, np.asarray(arrival) - arrival[0] if arrival else None)


def read_bag(path, topic, progress=None):
    """Read `topic` from a rosbag2 directory (sqlite3 or mcap)."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    rows = []
    while reader.has_next():
        _, data, _ = reader.read_next()
        rows.append(_row(deserialize_message(data, Imu)))
        if progress and len(rows) % 200000 == 0:
            progress(len(rows))
    if not rows:
        raise RuntimeError(f'No messages on {topic} in {path}')
    return _from_rows(rows)


def load(path, topic):
    """A `.npz` written by Recording.save, or a rosbag2 directory."""
    if str(path).endswith('.npz'):
        return Recording.load(path)
    return read_bag(path, topic)
