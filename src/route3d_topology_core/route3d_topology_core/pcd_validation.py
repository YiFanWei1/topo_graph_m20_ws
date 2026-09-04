"""Lightweight local point-cloud overlap validation for loop-closure candidates."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Dict, Tuple

import numpy as np

from .topology import PoseSample


@dataclass(frozen=True)
class PcdValidationConfig:
    enabled: bool = True
    required: bool = False
    voxel_size: float = 0.30
    minimum_overlap: float = 0.45
    search_radius: float = 0.60
    z_search_radius: float = 0.30
    minimum_range: float = 0.50
    maximum_range: float = 12.0
    minimum_voxels: int = 100

    def validate(self) -> None:
        positive = (
            self.voxel_size, self.search_radius, self.maximum_range)
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("PCD voxel size, search radius and maximum range must be positive")
        if not math.isfinite(self.z_search_radius) or self.z_search_radius < 0.0:
            raise ValueError("PCD z search radius must be non-negative")
        if not math.isfinite(self.minimum_range) or self.minimum_range < 0.0:
            raise ValueError("PCD minimum range must be non-negative")
        if self.minimum_range >= self.maximum_range:
            raise ValueError("PCD minimum range must be smaller than maximum range")
        if not 0.0 <= self.minimum_overlap <= 1.0:
            raise ValueError("PCD minimum overlap must be in [0, 1]")
        if self.minimum_voxels <= 0:
            raise ValueError("PCD minimum voxel count must be positive")


def _rotation_matrix(sample: PoseSample) -> np.ndarray:
    x, y, z, w = sample.quaternion
    return np.asarray([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)


def _read_binary_xyz(path: Path) -> np.ndarray:
    payload = path.read_bytes()
    marker = b"DATA binary"
    marker_offset = payload.find(marker)
    if marker_offset < 0:
        raise ValueError(f"only binary xyz PCD is supported: {path}")
    data_offset = payload.find(b"\n", marker_offset)
    if data_offset < 0:
        raise ValueError(f"invalid PCD header: {path}")
    header = payload[:data_offset].decode("ascii", errors="strict").splitlines()
    fields = next(
        (line.split()[1:] for line in header if line.upper().startswith("FIELDS ")),
        None)
    if fields != ["x", "y", "z"]:
        raise ValueError(f"PCD fields must be exactly x y z: {path}")
    raw = payload[data_offset + 1:]
    if len(raw) % 12 != 0:
        raise ValueError(f"invalid binary xyz payload length: {path}")
    return np.frombuffer(raw, dtype=np.float32).reshape((-1, 3)).astype(np.float64)


class PcdLoopClosureValidator:
    """Validate two colocated key frames using translation-tolerant voxel overlap."""

    def __init__(
        self, key_frames_directory: Path,
        config: PcdValidationConfig = PcdValidationConfig(),
    ) -> None:
        config.validate()
        self.directory = key_frames_directory.expanduser().resolve()
        self.config = config
        self._cache: Dict[Tuple[int, Tuple[float, ...]], frozenset[tuple[int, int, int]]] = {}

    def _unavailable(self, reason: str) -> Tuple[bool, dict]:
        return (not self.config.required), {
            "available": False,
            "accepted": not self.config.required,
            "reason": reason,
        }

    def _voxels(self, sample: PoseSample) -> frozenset[tuple[int, int, int]]:
        if sample.frame_index is None:
            raise ValueError("pose has no key-frame index")
        pose_key = tuple(round(value, 6) for value in (*sample.point, *sample.quaternion))
        key = (sample.frame_index, pose_key)
        if key in self._cache:
            return self._cache[key]
        path = self.directory / f"{sample.frame_index}.pcd"
        points = _read_binary_xyz(path)
        finite = np.isfinite(points).all(axis=1)
        radial = np.linalg.norm(points[:, :2], axis=1)
        points = points[
            finite & (radial >= self.config.minimum_range) &
            (radial <= self.config.maximum_range)]
        world = points @ _rotation_matrix(sample).T + np.asarray(sample.point)
        quantized = np.floor(world / self.config.voxel_size).astype(np.int32)
        unique = np.unique(quantized, axis=0)
        result = frozenset(tuple(int(value) for value in row) for row in unique)
        self._cache[key] = result
        return result

    def __call__(
        self, current: PoseSample, reference: PoseSample,
    ) -> Tuple[bool, dict]:
        if not self.config.enabled:
            return True, {"available": False, "accepted": True, "reason": "disabled"}
        if current.frame_index is None or reference.frame_index is None:
            return self._unavailable("missing_frame_index")
        current_path = self.directory / f"{current.frame_index}.pcd"
        reference_path = self.directory / f"{reference.frame_index}.pcd"
        if not current_path.is_file() or not reference_path.is_file():
            return self._unavailable("pcd_file_missing")
        try:
            current_voxels = self._voxels(current)
            reference_voxels = self._voxels(reference)
        except (OSError, ValueError) as error:
            return self._unavailable(str(error))
        if min(len(current_voxels), len(reference_voxels)) < self.config.minimum_voxels:
            return self._unavailable("too_few_voxels")

        xy_steps = int(math.ceil(self.config.search_radius / self.config.voxel_size))
        z_steps = int(math.ceil(self.config.z_search_radius / self.config.voxel_size))
        denominator = min(len(current_voxels), len(reference_voxels))
        best_overlap = 0.0
        best_shift = (0, 0, 0)
        for dx in range(-xy_steps, xy_steps + 1):
            for dy in range(-xy_steps, xy_steps + 1):
                for dz in range(-z_steps, z_steps + 1):
                    intersection = sum(
                        (x + dx, y + dy, z + dz) in reference_voxels
                        for x, y, z in current_voxels)
                    overlap = intersection / denominator
                    if overlap > best_overlap:
                        best_overlap = overlap
                        best_shift = (dx, dy, dz)
        accepted = best_overlap + 1.0e-12 >= self.config.minimum_overlap
        return accepted, {
            "available": True,
            "accepted": accepted,
            "overlap": best_overlap,
            "minimum_overlap": self.config.minimum_overlap,
            "current_voxels": len(current_voxels),
            "reference_voxels": len(reference_voxels),
            "best_translation_m": [
                value * self.config.voxel_size for value in best_shift],
        }
