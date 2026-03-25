"""
hambot_receiver.py - TCP Client: Raspberry Pi / HamBot Side
Overhead Perception System
Author: Aaron Fraze
Date: February 25, 2026

Purpose:
    Runs on the Raspberry Pi aboard HamBot.
    Connects to the overhead PC server, receives world state JSON
    every frame, and calls your robot behavior code with the parsed data.

    This file has NO dependency on RealSense, OpenCV, or NumPy.
    Only stdlib modules: socket, json, time, threading.
    Compatible with Python 3.11.2 and all Pi-side library versions.

Network:
    Connects to world_state_server.py on the overhead PC.
    Reconnects automatically if connection drops.

Usage:
    python hambot_receiver.py --server-ip 192.168.1.100

    Arguments:
        --server-ip   IP address of the overhead PC (required)
        --port        Port number (default 9999, must match server)

    Find your overhead PC's IP:
        Windows: ipconfig  (look for IPv4 Address on your WiFi adapter)
        e.g.    python hambot_receiver.py --server-ip 192.168.1.100

Behavior code:
    Edit the HambotBehavior class at the bottom of this file.
    on_state(state) is called every frame with the parsed world state dict.
    Call your existing motor/movement functions from there.

State dict structure (matches world_state.py FrameState.to_dict()):
    {
        "timestamp": 1740339600.123,   # Unix time seconds
        "frame_id":  4821,             # monotonically increasing
        "fps":       29.8,             # server-side FPS
        "robot": {
            "detected":         true,
            "x":                45.2,  # cm, world frame
            "y":               -12.8,
            "heading_deg":     127.4,  # 0=right, 90=forward, CCW positive
            "heading_current":  true,  # False = held from last ArUco frame
            "heading_age":      0,     # frames since last fresh ArUco heading
            "source":          "aruco",# "aruco", "hsv", or "lost"
            "confidence":       0.94
        },
        "ball": {
            "detected":  true,
            "x":         80.1,
            "y":         33.5,
            "vx":       -12.3,         # cm/s, world frame
            "vy":         4.1,
            "speed":     12.98,        # cm/s magnitude
            "confidence": 0.87
        },
        "goal": {
            "x": 110.0,
            "y":   0.0
        }
    }
"""

import argparse
import json
import socket
import sys
import time
import threading
from robot_systems.robot import HamBot


# ── Connection settings ───────────────────────────────────────────────────────
DEFAULT_PORT          = 9999
RECONNECT_DELAY_SEC   = 2.0    # seconds to wait before reconnect attempt
RECV_BUFFER_SIZE      = 4096   # bytes per socket read
CONNECTION_TIMEOUT    = 5.0    # seconds before connect() gives up


# ── Receiver ──────────────────────────────────────────────────────────────────

class WorldStateReceiver:
    """
    TCP client that receives world state JSON from the overhead server.

    Handles connection, reconnection, and message parsing.
    Calls behavior.on_state(state_dict) for every complete message received.

    Threading:
        Receive loop runs on the main thread.
        If you need motor commands on a separate thread, do that inside
        your HambotBehavior class -- the receiver itself is single-threaded.
    """

    def __init__(
        self,
        server_ip: str,
        port:      int,
        behavior,
    ):
        """
        Args:
            server_ip: IP address of the overhead PC running world_state_server.py
            port:      Must match server port (default 9999)
            behavior:  Object with on_state(dict) and on_disconnect() methods.
                       See HambotBehavior class below.
        """
        self.server_ip = server_ip
        self.port      = port
        self.behavior  = behavior
        self._running  = False

        # Stats
        self._messages_received: int   = 0
        self._connect_attempts:  int   = 0
        self._start_time:        float = 0.0

    def run(self):
        """
        Main loop. Connects to server, receives messages, reconnects on drop.
        Blocks until KeyboardInterrupt or stop() is called.
        """
        self._running    = True
        self._start_time = time.time()

        print(f"[Receiver] Connecting to {self.server_ip}:{self.port}")
        print(f"[Receiver] Press Ctrl+C to stop\n")

        try:
            while self._running:
                self._connect_attempts += 1
                sock = self._connect()

                if sock is None:
                    # Connection failed -- wait and retry
                    print(f"[Receiver] Retrying in {RECONNECT_DELAY_SEC}s...")
                    time.sleep(RECONNECT_DELAY_SEC)
                    continue

                print(f"[Receiver] Connected. Receiving world state...")
                self.behavior.on_connect()

                # Receive loop for this connection
                disconnected = self._receive_loop(sock)

                sock.close()
                self.behavior.on_disconnect()

                if disconnected and self._running:
                    print(f"[Receiver] Connection lost. Reconnecting...")
                    time.sleep(RECONNECT_DELAY_SEC)

        except KeyboardInterrupt:
            print("\n[Receiver] Stopped by user.")
        finally:
            self._running = False
            self._print_stats()
            bot.stop_motors()

    def stop(self):
        """Signal the receiver to stop after the current message."""
        self._running = False

    # ── Connection ────────────────────────────────────────────────────────────

    def _connect(self) -> socket.socket | None:
        """
        Attempt one TCP connection to the server.
        Returns connected socket or None on failure.
        """
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(CONNECTION_TIMEOUT)
            sock.connect((self.server_ip, self.port))
            sock.settimeout(None)   # blocking mode for recv loop
            return sock
        except (ConnectionRefusedError, socket.timeout, OSError) as e:
            print(f"[Receiver] Could not connect: {e}")
            return None

    # ── Receive loop ──────────────────────────────────────────────────────────

    def _receive_loop(self, sock: socket.socket) -> bool:
        """
        Read newline-delimited JSON messages from socket until disconnect.

        Returns True if disconnected (caller should reconnect),
                False if stopped intentionally.

        Uses makefile() to get a file-like object so we can use readline().
        This is the cleanest way to handle newline-delimited messages in Python.
        readline() blocks until a complete line arrives -- no manual buffering needed.
        """
        try:
            # makefile gives us readline() on a socket -- works identically
            # to reading from a file, handles partial reads internally
            sock_file = sock.makefile('r', encoding='utf-8')

            while self._running:
                line = sock_file.readline()

                if not line:
                    # Empty string from readline() means connection closed
                    return True

                line = line.strip()
                if not line:
                    continue

                # Parse JSON
                try:
                    state = json.loads(line)
                except json.JSONDecodeError as e:
                    print(f"[Receiver] JSON parse error: {e}")
                    continue

                self._messages_received += 1

                # Hand off to behavior code
                try:
                    self.behavior.on_state(state)
                except Exception as e:
                    # Don't let a behavior bug crash the receiver
                    print(f"[Receiver] Behavior error: {e}")

        except (ConnectionResetError, BrokenPipeError, OSError):
            return True   # disconnected

        return False  # stopped intentionally

    # ── Stats ─────────────────────────────────────────────────────────────────

    def _print_stats(self):
        elapsed = time.time() - self._start_time
        print(f"\n[Receiver] Session stats:")
        print(f"  Runtime           : {elapsed:.1f}s")
        print(f"  Messages received : {self._messages_received}")
        print(f"  Connect attempts  : {self._connect_attempts}")
        if elapsed > 0:
            print(f"  Avg receive rate  : {self._messages_received / elapsed:.1f} msg/s")


# ── HambotBehavior ────────────────────────────────────────────────────────────
# THIS IS WHERE YOU WRITE YOUR ROBOT CODE.
#
# on_state(state) is called every frame (~30x per second).
# Import and call your existing motor/movement scripts from here.
#
# Keep on_state() fast -- it runs on the receive thread.
# If you need longer-running behavior (e.g. a turn that takes 500ms),
# set a flag here and handle it in a separate thread or state machine.

class HambotBehavior:
    """
    Robot behavior driven by overhead world state.

    Edit this class to implement your autonomous behavior.
    Import your existing motor scripts at the top of this class.

    Example imports (update paths to match your Pi file structure):
        from hambot_motors import drive_forward, turn_left, stop
        from hambot_motors import set_speed, drive_arc
    """

    def __init__(self):
        # ── Import your motor scripts here ────────────────────────────────────
        # from hambot_motors import drive_forward, stop, turn_left, turn_right
        # self.motors = HambotMotors()

        # ── Behavior state ────────────────────────────────────────────────────
        self.state_name     = 'waiting'   # current behavior state
        self.frame_count    = 0
        self.last_print_t   = 0.0

        print("[Behavior] HambotBehavior initialized")
        print("[Behavior] Edit on_state() to add your robot logic")

    def on_connect(self):
        """Called once when connection to server is established."""
        print("[Behavior] Connected to overhead system -- starting behavior")
        self.state_name = 'searching'
        # self.motors.stop()   # ensure robot is stopped on connect

    def on_disconnect(self):
        """Called when connection to server drops."""
        print("[Behavior] Lost connection to overhead system -- stopping robot")
        self.state_name = 'waiting'
        # self.motors.stop()   # safety stop on disconnect -- IMPORTANT

    def on_state(self, state: dict):
        """
        Called every frame (~30x/sec) with the latest world state.

        Args:
            state: Parsed JSON dict from the overhead server.
                   See module docstring for full structure.

        TODO: Replace the placeholder logic below with your behavior.
        """
        # Just drive in circle for testing. Radius 50 cm
        # Hambot dimensions: wheelbase 184mm, wheels 90 mm.
        # @50cm radius> inner wheel radius = 40.8cm, outer wheel radius = 59.2cm
        # inner wheel circle=256.35cm, outer wheel circle=371.97cm
        # speed ratio = 1.45
        # set wheel speed once for continuous motion until program is stopped
        if self.frame_count < 1:
            bot.set_left_motor_speed(20)
            bot.set_right_motor_speed(29)
        self.frame_count += 1

        # ── Extract the data you need ─────────────────────────────────────────
        robot = state.get('robot', {})
        ball  = state.get('ball',  {})
        goal  = state.get('goal',  {})

        robot_detected  = robot.get('detected',        False)
        robot_x         = robot.get('x',               0.0)
        robot_y         = robot.get('y',               0.0)
        heading         = robot.get('heading_deg',     0.0)
        heading_current = robot.get('heading_current', False)
        heading_age     = robot.get('heading_age',     0)
        robot_source    = robot.get('source',          'lost')
        robot_conf      = robot.get('confidence',      0.0)

        ball_detected   = ball.get('detected',         False)
        ball_x          = ball.get('x',                0.0)
        ball_y          = ball.get('y',                0.0)
        ball_vx         = ball.get('vx',               0.0)
        ball_vy         = ball.get('vy',               0.0)
        ball_speed      = ball.get('speed',            0.0)

        goal_x          = goal.get('x',                0.0)
        goal_y          = goal.get('y',                0.0)

        server_fps      = state.get('fps',             0.0)

        # ── Heading staleness check ───────────────────────────────────────────
        # If heading_age > 10, slow down so ArUco can reacquire
        heading_stale = (not heading_current) and (heading_age > 10)

        # ── Placeholder behavior state machine ────────────────────────────────
        # Replace this with your actual navigation logic.
        # This is just a skeleton showing how to use the data.

        if not robot_detected:
            # Can't do anything useful if we don't know where we are
            self.state_name = 'lost'
            # self.motors.stop()

        elif heading_stale:
            # Heading unreliable -- slow down and let ArUco reacquire
            self.state_name = 'reacquiring_heading'
            # self.motors.set_speed(0.2)   # slow crawl

        elif not ball_detected:
            # Know where robot is but not ball -- search or hold position
            self.state_name = 'searching_ball'
            # self.motors.turn_left(speed=0.3)

        else:
            # Full state available -- implement ball-pushing behavior here
            self.state_name = 'pursuing_ball'

            # Useful values for navigation:
            #   dx_to_ball = ball_x - robot_x
            #   dy_to_ball = ball_y - robot_y
            #   dist_to_ball = sqrt(dx^2 + dy^2)
            #
            #   predicted ball position in N frames:
            #   pred_x = ball_x + ball_vx * (N / server_fps)
            #   pred_y = ball_y + ball_vy * (N / server_fps)
            #
            #   angle to ball from robot:
            #   import math
            #   angle_to_ball = math.degrees(math.atan2(dy_to_ball, dx_to_ball))
            #   turn_needed   = angle_to_ball - heading  (normalize to -180..180)

            # self.motors.drive_forward(speed=0.5)
            pass

        # ── Periodic console print (every 1 second) ───────────────────────────
        # Remove or reduce frequency once behavior is working
        now = time.time()
        if now - self.last_print_t >= 1.0:
            self.last_print_t = now
            print(
                f"[Behavior] state={self.state_name:20s} | "
                f"robot={'({:.1f},{:.1f})'.format(robot_x, robot_y) if robot_detected else 'LOST':15s} | "
                f"hdg={heading:.1f}° {'live' if heading_current else f'held+{heading_age}':10s} | "
                f"ball={'({:.1f},{:.1f})'.format(ball_x, ball_y) if ball_detected else 'none':15s} | "
                f"server={server_fps:.1f}fps"
            )


# ── Entry point ───────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description='HamBot World State Receiver')
    p.add_argument('--server-ip', required=True,
                   help='IP address of the overhead PC (e.g. 192.168.1.100)')
    p.add_argument('--port', default=DEFAULT_PORT, type=int,
                   help=f'Server port (default {DEFAULT_PORT})')
    return p.parse_args()


if __name__ == '__main__':
    args = parse_args()

    print("=" * 60)
    print("HamBot World State Receiver")
    print("=" * 60)
    print(f"  Server : {args.server_ip}:{args.port}")
    print("=" * 60)
    print()

    behavior = HambotBehavior()
    receiver = WorldStateReceiver(
        server_ip = args.server_ip,
        port      = args.port,
        behavior  = behavior,
    )
    bot = HamBot(lidar_enabled=False, camera_enabled=False)
    receiver.run()
