from __future__ import annotations

import math
import time
from typing import Optional, Sequence, Tuple

import numpy as np


DEFAULT_MULTIVIEW_CAMERAS = ("front", "side", "top")

_FONT = {
    "D": ("11110", "10001", "10001", "10001", "10001", "10001", "11110"),
    "E": ("11111", "10000", "10000", "11110", "10000", "10000", "11111"),
    "F": ("11111", "10000", "10000", "11110", "10000", "10000", "10000"),
    "I": ("11111", "00100", "00100", "00100", "00100", "00100", "11111"),
    "N": ("10001", "11001", "10101", "10011", "10001", "10001", "10001"),
    "O": ("01110", "10001", "10001", "10001", "10001", "10001", "01110"),
    "P": ("11110", "10001", "10001", "11110", "10000", "10000", "10000"),
    "R": ("11110", "10001", "10001", "11110", "10100", "10010", "10001"),
    "S": ("01111", "10000", "10000", "01110", "00001", "00001", "11110"),
    "T": ("11111", "00100", "00100", "00100", "00100", "00100", "00100"),
    "V": ("10001", "10001", "10001", "10001", "01010", "01010", "00100"),
    "W": ("10001", "10001", "10001", "10101", "10101", "10101", "01010"),
}


class ViewerCameraOverlay:
    def __init__(
        self,
        env,
        viewer,
        cameras: Sequence[str] = DEFAULT_MULTIVIEW_CAMERAS,
        layout: str = "grid",
        pane_width: int = 380,
        pane_height: int = 260,
        update_rate_hz: float = 15.0,
        margin: int = 12,
        gap: int = 8,
    ) -> None:
        if layout not in ("grid", "column"):
            raise ValueError("layout must be 'grid' or 'column'")
        self.env = env
        self.viewer = viewer
        self.cameras = tuple(cameras)
        self.layout = layout
        self.pane_width = int(pane_width)
        self.pane_height = int(pane_height)
        self.update_period = 1.0 / update_rate_hz if update_rate_hz > 0.0 else 0.0
        self.margin = int(margin)
        self.gap = int(gap)
        self._last_update = 0.0
        self._renderer = None
        self._renderer_size: Optional[Tuple[int, int]] = None
        self._validate_cameras()

    def sync(self) -> None:
        if not self.cameras:
            return
        now = time.monotonic()
        if self.update_period > 0.0 and now - self._last_update < self.update_period:
            return

        viewport = self.viewer.viewport
        if viewport is None or viewport.width <= 0 or viewport.height <= 0:
            return

        pane_width, pane_height, rects = self._layout_rects(viewport)
        renderer = self._ensure_renderer(pane_width, pane_height)
        viewports_images = []
        for camera, rect in zip(self.cameras, rects):
            renderer.update_scene(self.env.data, camera=camera)
            image = renderer.render()
            viewports_images.append((rect, _with_label(image, camera)))
        self.viewer.set_images(viewports_images)
        self._last_update = now

    def close(self) -> None:
        try:
            self.viewer.clear_images()
        except Exception:
            pass
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
            self._renderer_size = None

    def _validate_cameras(self) -> None:
        missing = []
        for camera in self.cameras:
            camera_id = self.env.mujoco.mj_name2id(
                self.env.model, self.env.mujoco.mjtObj.mjOBJ_CAMERA, camera
            )
            if camera_id < 0:
                missing.append(camera)
        if missing:
            raise RuntimeError(f"MuJoCo cameras not found: {missing}")

    def _ensure_renderer(self, width: int, height: int):
        size = (width, height)
        if self._renderer is not None and self._renderer_size == size:
            return self._renderer
        if self._renderer is not None:
            self._renderer.close()
        try:
            self._renderer = self.env.mujoco.Renderer(
                self.env.model,
                height=height,
                width=width,
            )
        except Exception as exc:
            raise RuntimeError(
                "Could not create MuJoCo multiview renderer. On macOS, run the "
                "viewer command with mjpython, or disable --multiview."
            ) from exc
        self._renderer_size = size
        return self._renderer

    def _layout_rects(self, viewport) -> Tuple[int, int, Tuple[object, ...]]:
        count = len(self.cameras)
        if self.layout == "column":
            columns = 1
        else:
            columns = min(2, count)
        rows = int(math.ceil(count / columns))

        width = max(96, self.pane_width)
        height = max(72, self.pane_height)
        total_width = columns * width + (columns - 1) * self.gap
        total_height = rows * height + (rows - 1) * self.gap

        scale = min(
            1.0,
            max(0.25, (viewport.width - 2 * self.margin) / max(total_width, 1)),
            max(0.25, (viewport.height - 2 * self.margin) / max(total_height, 1)),
        )
        width = max(64, int(width * scale))
        height = max(48, int(height * scale))
        total_width = columns * width + (columns - 1) * self.gap
        total_height = rows * height + (rows - 1) * self.gap

        left = max(0, int(viewport.width - self.margin - total_width))
        bottom = max(0, int(viewport.height - self.margin - total_height))

        rects = []
        for index in range(count):
            row = index // columns
            column = index % columns
            rect_left = left + column * (width + self.gap)
            rect_bottom = bottom + (rows - row - 1) * (height + self.gap)
            rects.append(self.env.mujoco.MjrRect(rect_left, rect_bottom, width, height))
        return width, height, tuple(rects)


def _with_label(image: np.ndarray, label: str) -> np.ndarray:
    labeled = np.array(image, copy=True)
    _draw_border(labeled)
    scale = 2 if min(labeled.shape[:2]) >= 120 else 1
    text_width = max(1, len(label)) * 6 * scale + 10
    text_height = 7 * scale + 8
    patch_width = min(labeled.shape[1], text_width)
    patch_height = min(labeled.shape[0], text_height)
    labeled[:patch_height, :patch_width] = (
        labeled[:patch_height, :patch_width].astype(np.float32) * 0.35
    ).astype(labeled.dtype)
    _draw_text(labeled, label.upper(), 5, 4, scale=scale)
    return labeled


def _draw_border(
    image: np.ndarray,
    color: Tuple[int, int, int] = (170, 178, 186),
    thickness: int = 2,
) -> None:
    image[:thickness, :] = color
    image[-thickness:, :] = color
    image[:, :thickness] = color
    image[:, -thickness:] = color


def _draw_text(
    image: np.ndarray,
    text: str,
    x: int,
    y: int,
    scale: int = 1,
    color: Tuple[int, int, int] = (245, 245, 245),
) -> None:
    cursor = int(x)
    for char in text:
        pattern = _FONT.get(char)
        if pattern is None:
            cursor += 4 * scale
            continue
        for row, line in enumerate(pattern):
            for column, value in enumerate(line):
                if value != "1":
                    continue
                top = y + row * scale
                left = cursor + column * scale
                image[top : top + scale, left : left + scale] = color
        cursor += 6 * scale
