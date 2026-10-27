"""Read-only CSV archive and analysis for the existing Python GUI.

Accepts both historical x_m/... files and the C++ target/actual format. No flight
records are written, deleted or merged here. RMSE never crosses an EKF reset.
"""
import csv
import io
import math
from pathlib import Path
from .config import ROOT


def _finite(row, names):
    values = [float(row[name]) for name in names]
    if not all(math.isfinite(value) for value in values):
        raise ValueError('nonfinite coordinate')
    return values


def analyze_csv(text, filename='import.csv'):
    if not isinstance(text, str) or not text.strip():
        raise ValueError('CSV为空')
    if len(text.encode('utf-8')) > 12 * 1024 * 1024:
        raise ValueError('CSV超过12MB，请分段导入')
    reader = csv.DictReader(io.StringIO(text.lstrip('\ufeff')))
    fields = set(reader.fieldnames or [])
    names = ('actual_x', 'actual_y', 'actual_z') if {'actual_x', 'actual_y', 'actual_z'} <= fields else ('x_m', 'y_m', 'z_m')
    if not set(names) <= fields:
        raise ValueError('CSV需要actual_x/y/z或x_m/y_m/z_m列')
    points, total, skipped, previous_time = [], 0, False, None
    segments, dropped, mid_session = 0, 0, False
    sums = [0., 0., 0.]
    errors = []
    valid_comparisons = 0
    for row in reader:
        total += 1
        if total > 200000:
            raise ValueError('CSV超过200000点，请分段导入')
        try:
            xyz = _finite(row, names)
            elapsed = float(row.get('elapsed_s') or row.get('timestamp') or total - 1)
            if not math.isfinite(elapsed):
                raise ValueError()
            segment = int(row.get('segment') or 0)
            sample = int(float(row.get('sample_ms') or elapsed * 1000))
            dropped = max(dropped, int(row.get('dropped_events') or 0))
        except (ValueError, KeyError, TypeError):
            skipped = True
            continue
        mid_session |= row.get('started_mid_session') == '1'
        valid = row.get('position_valid', '1') == '1'
        point = dict(xyz=xyz, sample_ms=sample, elapsed_s=elapsed, segment=segment,
                     break_before=skipped or (previous_time is not None and elapsed < previous_time), valid=valid,
                     yaw_deg=None, target=None, error=None)
        try:
            target = _finite(row, ('target_x', 'target_y', 'target_z'))
            point['target'] = target
            error = [a - b for a, b in zip(xyz, target)]
            # Invalid EKF points can be inspected, but must not improve error statistics.
            if valid:
                point['error'] = math.sqrt(sum(value * value for value in error))
                errors.append(point['error'])
                valid_comparisons += 1
                sums = [total_square + value * value for total_square, value in zip(sums, error)]
        except (ValueError, KeyError, TypeError):
            pass
        try:
            heading = float(row.get('yaw_deg') or 'nan')
            if math.isfinite(heading) and float(row.get('attitude_age_s') or 'inf') < .5:
                point['yaw_deg'] = heading
        except ValueError:
            pass
        if not points or segment != points[-1]['segment'] or point['break_before']:
            segments += 1
        points.append(point)
        previous_time, skipped = elapsed, False
    rmse_xyz = [math.sqrt(value / valid_comparisons) for value in sums] if valid_comparisons else None
    analysis = dict(compared_points=valid_comparisons, rmse_xyz_m=rmse_xyz,
                    rmse_3d_m=math.sqrt(sum(sums) / valid_comparisons) if valid_comparisons else None,
                    max_error_m=max(errors) if errors else None,
                    mean_error_m=sum(errors) / len(errors) if errors else None,
                    invalid_points=sum(not point['valid'] for point in points), segments=segments,
                    note='仅比较同一行的有效实际值和目标值；历史CSV无目标列时RMSE不可用。坐标为LOCAL_NED米。')
    return dict(filename=Path(filename).name, total_points=total, plotted_points=len(points),
                started_mid_session=mid_session, dropped_events=dropped, points=points, analysis=analysis)


class FlightArchive:
    def __init__(self, directory=None):
        self.directory = Path(directory) if directory else ROOT / 'flight_records'

    def latest_file(self):
        files = [path for path in self.directory.glob('flight_*.csv')
                 if not path.name.endswith(('.partial.csv', '.interrupted.csv'))]
        if not files:
            raise FileNotFoundError('暂无完整CSV；C++服务在解锁后记录、确认上锁后完成文件')
        return max(files, key=lambda path: path.name)

    def status(self):
        try:
            latest = self.latest_file().name
        except FileNotFoundError:
            latest = None
        return dict(recording=False, latest_complete=latest, writer='cpp')

    def latest_trajectory(self):
        path = self.latest_file()
        return analyze_csv(path.read_text(encoding='utf-8'), path.name)
