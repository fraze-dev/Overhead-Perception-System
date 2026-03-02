"""
world_state.py - World State Estimator
Overhead Perception System
Author: Aaron Fraze
Date: February 25, 2026

Purpose:
    Single integration point for all detectors. Each frame, WorldState:
        1. Runs ArUco detector  (robot primary)
        2. Runs HSV detector    (robot fallback + ball)
        3. Applies ArUco -> HSV fallback logic for robot
        4. Holds last-known heading with age counter
        5. Estimates ball velocity from position history
        6. Packages everything into a clean state dict ready for TCP send

    This is the only class the rest of the system talks to.
    Detectors are internal implementation details.

Usage:
    from camera import RealSenseCamera
    from world_state import WorldState

    cam   = RealSenseCamera(camera_height_cm=220.0)
    world = WorldState(cam, goal_position=(110.0, 0.0))

    while True:
        frame_data = cam.get_frame()
        state      = world.update(frame_data)
        print(state.robot)   # RobotState dataclass
        print(state.ball)    # BallState dataclass
        print(state.to_dict())  # JSON-ready dict for TCP send

Design decisions:
    - ArUco is primary robot detector (conf >= ARUCO_CONF_THRESHOLD)
    - HSV green is fallback when ArUco conf drops or marker not found
    - Heading always populated (hold-last-known when ArUco lost)
    - heading_current flag + heading_age counter tell robot how stale heading is
    - Ball velocity computed from exponential moving average of frame deltas
    - Goal position is static, set once at init, always included in state
    - to_dict() output is the exact JSON message sent over TCP
"""

import time
from collections import deque
from dataclasses import dataclass, field, asdict

import numpy as np

from camera import RealSenseCamera
from aruco_detector import ArucoDetector
from hsv_detector import HsvDetector


# ── Tunable constants ─────────────────────────────────────────────────────────

# ArUco confidence below this -> fall back to HSV for robot position
ARUCO_CONF_THRESHOLD = 0.50

# HSV robot confidence below this -> robot is considered lost entirely
HSV_CONF_THRESHOLD = 0.30

# Heading age above this (frames) -> warn robot heading is stale
HEADING_STALE_FRAMES = 10

# Ball velocity: exponential moving average alpha
# Higher = responds faster to direction changes, noisier
# Lower  = smoother velocity estimate, lags behind real changes
# 0.3 is a good starting point; tune after seeing real ball motion
VELOCITY_EMA_ALPHA = 0.3

# Ball position history depth (frames) used for velocity estimation
BALL_HISTORY_LEN = 10

# Minimum ball displacement (cm) between frames to count as real motion
# Below this is treated as sensor noise, velocity set to 0
BALL_MIN_MOTION_CM = 0.5


# ── State dataclasses ─────────────────────────────────────────────────────────
# Dataclasses give us:
#   - Free __repr__ for easy print debugging
#   - asdict() for clean JSON serialization
#   - Type hints for IDE support
#   - Immutable-feel structure (we create a new one each frame)

@dataclass
class RobotState:
    """
    Robot position, heading, and detection metadata for one frame.

    Fields the robot cares about for autonomous behavior:
        x, y            world position in cm
        heading_deg     0=right, 90=forward, increases CCW
        heading_current True if heading came from ArUco this frame
        heading_age     frames since last fresh ArUco heading (0 = this frame)
        detected        False if robot is completely lost this frame
        source          'aruco', 'hsv', or 'lost'
        confidence      0.0-1.0 detection confidence
    """
    detected:         bool  = False
    x:                float = 0.0
    y:                float = 0.0
    heading_deg:      float = 0.0
    heading_current:  bool  = False   # True = fresh ArUco heading this frame
    heading_age:      int   = 0       # frames since last fresh ArUco heading
    source:           str   = 'lost'  # 'aruco', 'hsv', 'lost'
    confidence:       float = 0.0


@dataclass
class BallState:
    """
    Ball position and velocity for one frame.

    vx, vy are in cm/s in world frame (+X right, +Y forward).
    speed is magnitude of velocity vector.
    Robot can use vx/vy to predict where ball will be in N frames:
        predicted_x = ball.x + ball.vx * (N / fps)
        predicted_y = ball.y + ball.vy * (N / fps)
    """
    detected:    bool  = False
    x:           float = 0.0
    y:           float = 0.0
    vx:          float = 0.0    # cm/s, world frame
    vy:          float = 0.0    # cm/s, world frame
    speed:       float = 0.0    # cm/s, magnitude
    confidence:  float = 0.0


@dataclass
class GoalState:
    """
    Goal region. Static — set once at WorldState init."""
    x: float = 0.0
    y: float = 0.0


@dataclass
class FrameState:
    """
    Complete world state for one frame. This is what gets serialized to JSON
    and sent over TCP to the robot every frame.
    """
    timestamp:  float      = 0.0   # Unix time, seconds
    frame_id:   int        = 0     # monotonically increasing frame counter
    fps:        float      = 0.0   # current processing FPS
    robot:      RobotState = field(default_factory=RobotState)
    ball:       BallState  = field(default_factory=BallState)
    goal:       GoalState  = field(default_factory=GoalState)

    def to_dict(self) -> dict:
        """
        Convert to a JSON-serializable dict.
        This is the exact payload sent over TCP.
        """
        return asdict(self)


# ── WorldState ────────────────────────────────────────────────────────────────

class WorldState:
    """
    Integrates all detectors and maintains state across frames.

    Instantiate once at startup, call update() every frame.
    """

    def __init__(
        self,
        camera: RealSenseCamera,
        goal_position: tuple = (110.0, 0.0),
        robot_marker_id: int = 1,
        aruco_conf_threshold: float = ARUCO_CONF_THRESHOLD,
        hsv_conf_threshold: float = HSV_CONF_THRESHOLD,
    ):
        """
        Args:
            camera:               Initialized RealSenseCamera instance.
            goal_position:        (x_cm, y_cm) of goal centre in world frame.
                                  Measure this once from your arena setup.
                                  Default is a placeholder — update for your arena.
            robot_marker_id:      ArUco marker ID mounted on HamBot.
                                  Must match the ID printed on your marker.
            aruco_conf_threshold: ArUco confidence below which HSV fallback
                                  is used for robot position.
            hsv_conf_threshold:   HSV confidence below which robot is 'lost'.
        """
        self.cam                  = camera
        self.goal                 = GoalState(x=goal_position[0], y=goal_position[1])
        self.robot_marker_id      = robot_marker_id
        self.aruco_conf_threshold = aruco_conf_threshold
        self.hsv_conf_threshold   = hsv_conf_threshold

        # ── Detectors ─────────────────────────────────────────────────────────
        self.aruco = ArucoDetector(camera)
        self.hsv   = HsvDetector(camera)

        # ── Heading state (persists across frames) ────────────────────────────
        self._last_heading_deg: float = 0.0    # last known good ArUco heading
        self._heading_age:      int   = 0      # frames since last ArUco heading

        # ── Ball velocity state ───────────────────────────────────────────────
        # Store recent ball positions with timestamps for velocity estimation
        self._ball_history: deque = deque(maxlen=BALL_HISTORY_LEN)
        self._ball_vx_ema:  float = 0.0
        self._ball_vy_ema:  float = 0.0

        # ── FPS tracking ──────────────────────────────────────────────────────
        self._frame_id:    int   = 0
        self._fps_history: deque = deque(maxlen=30)
        self._last_time:   float = time.time()

        print(f"[WorldState] Initialized")
        print(f"  Robot marker ID : {robot_marker_id}")
        print(f"  Goal position   : {goal_position} cm")
        print(f"  ArUco threshold : {aruco_conf_threshold}")
        print(f"  HSV threshold   : {hsv_conf_threshold}")

    # ── Main update loop ──────────────────────────────────────────────────────

    def update(self, frame_data: dict) -> FrameState:
        """
        Run all detectors on one frame and return complete world state.

        Call this every frame in your main loop:
            frame_data = cam.get_frame()
            state      = world.update(frame_data)

        Args:
            frame_data: dict from RealSenseCamera.get_frame()

        Returns:
            FrameState dataclass with robot, ball, goal, and metadata.
            Call state.to_dict() to get the JSON-ready payload.
        """
        now = time.time()

        # ── FPS ───────────────────────────────────────────────────────────────
        dt = now - self._last_time
        self._last_time = now
        if dt > 0:
            self._fps_history.append(1.0 / dt)
        fps = float(np.mean(self._fps_history)) if self._fps_history else 0.0

        self._frame_id += 1

        # ── Run detectors ─────────────────────────────────────────────────────
        aruco_detections = self.aruco.detect(frame_data)
        hsv_detections   = self.hsv.detect(frame_data)

        # ── Build robot state ─────────────────────────────────────────────────
        robot_state = self._build_robot_state(aruco_detections, hsv_detections)

        # ── Build ball state ──────────────────────────────────────────────────
        ball_state = self._build_ball_state(hsv_detections['ball'], now)

        return FrameState(
            timestamp = now,
            frame_id  = self._frame_id,
            fps       = round(fps, 1),
            robot     = robot_state,
            ball      = ball_state,
            goal      = self.goal,
        )

    # ── Robot state builder ───────────────────────────────────────────────────

    def _build_robot_state(
        self,
        aruco_detections: list,
        hsv_detections:   dict,
    ) -> RobotState:
        """
        Build RobotState applying ArUco -> HSV fallback logic.

        Priority:
            1. ArUco detection for robot_marker_id with conf >= threshold
               -> position + fresh heading
            2. HSV green detection with conf >= threshold
               -> position only, hold last known heading
            3. Neither found / both below threshold
               -> robot lost, hold last known position and heading

        Heading hold logic:
            heading_deg     always populated (last known value)
            heading_current True only when ArUco gave a fresh heading this frame
            heading_age     increments every frame ArUco is not found,
                            resets to 0 when ArUco gives a fresh heading
        """

        # ── Try ArUco first ───────────────────────────────────────────────────
        aruco_robot = None
        for det in aruco_detections:
            if det['id'] == self.robot_marker_id and \
               det['confidence'] >= self.aruco_conf_threshold:
                aruco_robot = det
                break

        if aruco_robot is not None:
            pos = aruco_robot['position_world']
            x   = pos[0] if pos else 0.0
            y   = pos[1] if pos else 0.0

            # Fresh ArUco heading — update stored heading, reset age
            self._last_heading_deg = aruco_robot['heading_deg']
            self._heading_age      = 0

            return RobotState(
                detected        = True,
                x               = round(x, 2),
                y               = round(y, 2),
                heading_deg     = round(self._last_heading_deg, 2),
                heading_current = True,
                heading_age     = 0,
                source          = 'aruco',
                confidence      = round(aruco_robot['confidence'], 3),
            )

        # ── ArUco failed — try HSV green fallback ─────────────────────────────
        self._heading_age += 1   # ArUco not found this frame

        hsv_robot = hsv_detections.get('robot')

        if hsv_robot is not None and \
           hsv_robot['confidence'] >= self.hsv_conf_threshold:
            pos = hsv_robot['position_world']
            x   = pos[0] if pos else 0.0
            y   = pos[1] if pos else 0.0

            # Position from HSV, heading held from last ArUco
            return RobotState(
                detected        = True,
                x               = round(x, 2),
                y               = round(y, 2),
                heading_deg     = round(self._last_heading_deg, 2),
                heading_current = False,               # heading is held, not fresh
                heading_age     = self._heading_age,
                source          = 'hsv',
                confidence      = round(hsv_robot['confidence'], 3),
            )

        # ── Both failed — robot lost ───────────────────────────────────────────
        # Return last known position (0,0 on first frame) with lost status.
        # Robot behavior code should treat 'lost' as "stop and wait".
        return RobotState(
            detected        = False,
            x               = 0.0,
            y               = 0.0,
            heading_deg     = round(self._last_heading_deg, 2),
            heading_current = False,
            heading_age     = self._heading_age,
            source          = 'lost',
            confidence      = 0.0,
        )

    # ── Ball state builder ────────────────────────────────────────────────────

    def _build_ball_state(
        self,
        hsv_ball,
        timestamp: float,
    ) -> BallState:
        """
        Build BallState with velocity from exponential moving average.

        Velocity estimation:
            Each frame we compute instantaneous velocity from the position
            delta between this frame and the previous frame, divided by dt.
            We then apply EMA smoothing to reduce noise:
                vx_ema = alpha * vx_instant + (1 - alpha) * vx_ema_prev

            This gives a smooth velocity estimate that still responds
            quickly to real direction changes.

        If ball is not detected this frame:
            - velocity history is preserved (ball may reappear next frame)
            - returned BallState has detected=False and speed=0
            - x,y hold the last known position
        """

        if hsv_ball is None:
            # Ball not found this frame — return not-detected state
            # Don't clear velocity history; ball may reappear immediately
            return BallState(
                detected   = False,
                confidence = 0.0,
            )

        pos = hsv_ball.get('position_world')
        if pos is None:
            return BallState(detected=False, confidence=0.0)

        bx, by = pos[0], pos[1]

        # ── Velocity estimation ───────────────────────────────────────────────
        if len(self._ball_history) >= 1:
            prev_t, prev_x, prev_y = self._ball_history[-1]
            dt = timestamp - prev_t

            if dt > 0:
                dx = bx - prev_x
                dy = by - prev_y
                displacement = np.sqrt(dx**2 + dy**2)

                if displacement >= BALL_MIN_MOTION_CM:
                    # Real motion — compute instantaneous velocity and smooth
                    vx_inst = dx / dt
                    vy_inst = dy / dt
                    self._ball_vx_ema = (VELOCITY_EMA_ALPHA * vx_inst +
                                         (1 - VELOCITY_EMA_ALPHA) * self._ball_vx_ema)
                    self._ball_vy_ema = (VELOCITY_EMA_ALPHA * vy_inst +
                                         (1 - VELOCITY_EMA_ALPHA) * self._ball_vy_ema)
                else:
                    # Displacement below noise threshold — decay velocity toward 0
                    # This prevents phantom velocity when ball is stationary
                    self._ball_vx_ema *= (1 - VELOCITY_EMA_ALPHA)
                    self._ball_vy_ema *= (1 - VELOCITY_EMA_ALPHA)

        # Store this frame in history
        self._ball_history.append((timestamp, bx, by))

        speed = float(np.sqrt(self._ball_vx_ema**2 + self._ball_vy_ema**2))

        return BallState(
            detected   = True,
            x          = round(bx, 2),
            y          = round(by, 2),
            vx         = round(self._ball_vx_ema, 2),
            vy         = round(self._ball_vy_ema, 2),
            speed      = round(speed, 2),
            confidence = round(hsv_ball['confidence'], 3),
        )

    # ── Visualisation ─────────────────────────────────────────────────────────

    def draw_world_state(self, image: np.ndarray, state: FrameState) -> np.ndarray:
        """
        Draw the full world state onto a camera image.

        Shows robot position/heading, ball position/velocity vector,
        and goal marker. Useful as the main debug visualisation window.

        Args:
            image: BGR image from frame_data['color_image']
            state: FrameState from update()

        Returns:
            Annotated BGR image.
        """
        import cv2
        vis = image.copy()
        h, w = vis.shape[:2]

        # ── HUD helper ────────────────────────────────────────────────────────
        def hud(text, row, color=(220, 220, 220)):
            cv2.putText(vis, text, (10, 25 + row * 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1)

        # ── Robot ─────────────────────────────────────────────────────────────
        r = state.robot
        if r.detected:
            src_color = {
                'aruco': (0, 255, 100),
                'hsv':   (0, 200, 255),
                'lost':  (0, 0, 200),
            }.get(r.source, (200, 200, 200))

            hud(f"ROBOT [{r.source.upper()}]  ({r.x:.1f}, {r.y:.1f}) cm", 0, src_color)
            hud(f"  heading: {r.heading_deg:.1f}°  "
                f"{'live' if r.heading_current else f'held {r.heading_age}fr'}",
                1, src_color)
            hud(f"  conf: {r.confidence:.2f}", 2, src_color)

            # Heading staleness warning
            if not r.heading_current and r.heading_age > HEADING_STALE_FRAMES:
                hud(f"  ⚠ HEADING STALE ({r.heading_age} frames)", 3, (0, 80, 255))
        else:
            hud("ROBOT: LOST", 0, (0, 0, 255))
            hud(f"  last heading: {r.heading_deg:.1f}° ({r.heading_age} frames ago)",
                1, (100, 100, 100))

        # ── Ball ──────────────────────────────────────────────────────────────
        b = state.ball
        if b.detected:
            hud(f"BALL  ({b.x:.1f}, {b.y:.1f}) cm", 4, (0, 165, 255))
            hud(f"  vel: ({b.vx:.1f}, {b.vy:.1f}) cm/s  speed={b.speed:.1f}", 5, (0, 165, 255))
            hud(f"  conf: {b.confidence:.2f}", 6, (0, 165, 255))
        else:
            hud("BALL: not detected", 4, (100, 100, 100))

        # ── Goal ──────────────────────────────────────────────────────────────
        g = state.goal
        hud(f"GOAL  ({g.x:.1f}, {g.y:.1f}) cm", 7, (255, 255, 100))

        # ── Frame metadata ────────────────────────────────────────────────────
        hud(f"FPS: {state.fps:.1f}  frame: {state.frame_id}", 8, (150, 150, 150))

        return vis

    # ── Diagnostic print ──────────────────────────────────────────────────────

    def print_state(self, state: FrameState):
        """
        Print a concise one-line state summary to console.
        Useful for logging and debugging without opening a window.
        """
        r = state.robot
        b = state.ball

        robot_str = (
            f"robot=[{r.source}] ({r.x:.1f},{r.y:.1f}) "
            f"hdg={r.heading_deg:.1f}° "
            f"{'live' if r.heading_current else f'held+{r.heading_age}'} "
            f"conf={r.confidence:.2f}"
            if r.detected else "robot=LOST"
        )

        ball_str = (
            f"ball=({b.x:.1f},{b.y:.1f}) "
            f"v=({b.vx:.1f},{b.vy:.1f}) spd={b.speed:.1f}"
            if b.detected else "ball=none"
        )

        print(f"[{state.frame_id:05d}] {state.fps:.1f}fps | {robot_str} | {ball_str}")


# ── Standalone demo ───────────────────────────────────────────────────────────
if __name__ == '__main__':
    import cv2

    print("=" * 60)
    print("world_state.py — Live Demo")
    print("=" * 60)
    print("Shows integrated robot + ball state each frame.")
    print("Press 'p' to print state to console  'q' to quit")
    print()

    cam   = RealSenseCamera(resolution='1280x720', camera_height_cm=220.0)
    world = WorldState(
        cam,
        goal_position    = (110.0, 0.0),   # update for your arena
        robot_marker_id  = 1,              # update for your marker ID
    )

    cv2.namedWindow('World State', cv2.WINDOW_NORMAL)

    try:
        while True:
            frame_data = cam.get_frame()
            if frame_data is None:
                continue

            state = world.update(frame_data)
            vis   = world.draw_world_state(frame_data['color_image'], state)

            cv2.imshow('World State', vis)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('p'):
                world.print_state(state)

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        cv2.destroyAllWindows()
        cam.shutdown()
