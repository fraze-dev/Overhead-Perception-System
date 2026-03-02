"""
world_state_server.py - TCP Server: Overhead PC Side
Overhead Perception System
Author: Aaron Fraze
Date: February 25, 2026

Purpose:
    Runs on the overhead Windows PC.
    Each frame:
        1. Captures a frame from the RealSense camera
        2. Calls WorldState.update() -- runs all detectors
        3. Serializes FrameState to JSON
        4. Sends to connected robot client over TCP

    Handles one client connection at a time (one robot).
    If the robot disconnects, server waits for reconnection
    without restarting the camera or losing detector state.

Network:
    Protocol : TCP
    Default port : 9999
    Message format : JSON + newline  e.g.  {"timestamp": ...}\n
    One message per frame (~30/sec)

Usage:
    python world_state_server.py

    Optional arguments:
        --host      IP to bind to (default 0.0.0.0 = all interfaces)
        --port      Port number   (default 9999)
        --marker-id ArUco marker ID on HamBot (default 1)
        --goal-x    Goal X position in cm (default 110.0)
        --goal-y    Goal Y position in cm (default 0.0)
        --no-display  Run headless, no OpenCV window

    Example with custom settings:
        python world_state_server.py --port 9999 --marker-id 1 --goal-x 110 --goal-y 0
"""

import argparse
import json
import socket
import sys
import threading
import time

import cv2

from camera import RealSenseCamera
from world_state import WorldState


# ── Defaults ──────────────────────────────────────────────────────────────────
DEFAULT_HOST      = '0.0.0.0'   # listen on all network interfaces
DEFAULT_PORT      = 9999
DEFAULT_MARKER_ID = 1
DEFAULT_GOAL_X    = 110.0
DEFAULT_GOAL_Y    = 0.0


# ── Server ────────────────────────────────────────────────────────────────────

class WorldStateServer:
    """
    TCP server that streams world state JSON to a connected robot client.

    Threading model:
        Main thread  : camera capture + detection + display (needs to be fast)
        Send thread  : blocking socket send (isolated so a slow network
                       write never stalls the detection loop)

    The latest state dict is stored in self._pending and the send thread
    picks it up. If the network is slower than the camera, frames are
    dropped gracefully -- the robot always gets the LATEST state, never
    a queue of stale ones.
    """

    def __init__(
        self,
        host:          str   = DEFAULT_HOST,
        port:          int   = DEFAULT_PORT,
        marker_id:     int   = DEFAULT_MARKER_ID,
        goal_position: tuple = (DEFAULT_GOAL_X, DEFAULT_GOAL_Y),
        show_display:  bool  = True,
    ):
        self.host          = host
        self.port          = port
        self.show_display  = show_display

        # ── Camera + world state ──────────────────────────────────────────────
        print("[Server] Initializing camera...")
        self.cam   = RealSenseCamera(resolution='1280x720', camera_height_cm=220.0)
        self.world = WorldState(
            self.cam,
            goal_position   = goal_position,
            robot_marker_id = marker_id,
        )

        # ── Connection state ──────────────────────────────────────────────────
        self._client_conn:   socket.socket | None = None
        self._client_addr:   tuple | None         = None
        self._connected:     bool                 = False

        # ── Latest state for send thread ──────────────────────────────────────
        self._pending:       dict | None          = None
        self._pending_lock:  threading.Lock       = threading.Lock()

        # ── Stats ─────────────────────────────────────────────────────────────
        self._frames_sent:   int  = 0
        self._frames_dropped: int = 0  # frames where client was too slow
        self._start_time:    float = time.time()

    # ── Main entry point ──────────────────────────────────────────────────────

    def run(self):
        """
        Start the server. Blocks until KeyboardInterrupt.

        Flow:
            1. Open TCP server socket
            2. Wait for robot to connect
            3. Start send thread
            4. Loop: capture frame -> detect -> display -> store for send
            5. On disconnect: stop send thread, wait for reconnect
        """
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Allow rapid restart without "address already in use" error
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind((self.host, self.port))
        server_sock.listen(1)
        server_sock.settimeout(1.0)  # non-blocking accept so Ctrl+C works

        print(f"\n[Server] Listening on {self.host}:{self.port}")
        print(f"[Server] Waiting for robot to connect...")
        print(f"[Server] Press Ctrl+C to stop\n")

        if self.show_display:
            cv2.namedWindow('World State Server', cv2.WINDOW_NORMAL)

        try:
            while True:
                # ── Wait for robot connection ─────────────────────────────────
                self._client_conn = None
                self._connected   = False

                while not self._connected:
                    try:
                        conn, addr = server_sock.accept()
                        self._client_conn = conn
                        self._client_addr = addr
                        self._connected   = True
                        print(f"[Server] Robot connected from {addr}")
                    except socket.timeout:
                        # No connection yet -- check for Ctrl+C and keep waiting
                        if self.show_display and cv2.waitKey(1) & 0xFF == ord('q'):
                            raise KeyboardInterrupt

                # ── Start send thread for this connection ─────────────────────
                send_thread = threading.Thread(
                    target=self._send_loop,
                    daemon=True
                )
                send_thread.start()

                # ── Detection + display loop ───────────────────────────────────
                self._run_detection_loop()

                # Robot disconnected -- send thread will exit, loop back to wait
                print(f"[Server] Robot disconnected. Waiting for reconnect...")
                send_thread.join(timeout=2.0)

        except KeyboardInterrupt:
            print("\n[Server] Shutting down...")
        finally:
            if self._client_conn:
                self._client_conn.close()
            server_sock.close()
            if self.show_display:
                cv2.destroyAllWindows()
            self.cam.shutdown()
            self._print_stats()

    # ── Detection loop (main thread) ──────────────────────────────────────────

    def _run_detection_loop(self):
        """
        Capture frames and run detectors until robot disconnects.
        Stores latest state dict for the send thread.
        """
        while self._connected:
            frame_data = self.cam.get_frame()
            if frame_data is None:
                continue

            # Run all detectors via WorldState
            state     = self.world.update(frame_data)
            state_dict = state.to_dict()

            # Store for send thread (drop previous unsent frame if any)
            with self._pending_lock:
                if self._pending is not None:
                    self._frames_dropped += 1
                self._pending = state_dict

            # Display
            if self.show_display:
                vis = self.world.draw_world_state(
                    frame_data['color_image'], state
                )
                # Connection status overlay
                cv2.putText(
                    vis,
                    f"CLIENT: {self._client_addr}  "
                    f"sent={self._frames_sent}  "
                    f"dropped={self._frames_dropped}",
                    (10, vis.shape[0] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 0), 1
                )
                cv2.imshow('World State Server', vis)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self._connected = False
                    break

    # ── Send loop (background thread) ─────────────────────────────────────────

    def _send_loop(self):
        """
        Runs in a background thread.
        Picks up the latest pending state dict and sends it over TCP.
        Exits when connection drops or self._connected becomes False.
        """
        while self._connected:
            state_dict = None

            with self._pending_lock:
                if self._pending is not None:
                    state_dict    = self._pending
                    self._pending = None

            if state_dict is None:
                # Nothing ready yet -- yield briefly and check again
                time.sleep(0.001)
                continue

            try:
                message = json.dumps(state_dict) + '\n'
                self._client_conn.sendall(message.encode('utf-8'))
                self._frames_sent += 1
            except (BrokenPipeError, ConnectionResetError, OSError):
                # Robot disconnected
                self._connected = False
                break

    # ── Stats ─────────────────────────────────────────────────────────────────

    def _print_stats(self):
        elapsed = time.time() - self._start_time
        print(f"\n[Server] Session stats:")
        print(f"  Runtime        : {elapsed:.1f}s")
        print(f"  Frames sent    : {self._frames_sent}")
        print(f"  Frames dropped : {self._frames_dropped}")
        if elapsed > 0:
            print(f"  Avg send rate  : {self._frames_sent / elapsed:.1f} fps")


# ── Entry point ───────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description='Overhead World State TCP Server')
    p.add_argument('--host',       default=DEFAULT_HOST,      help='IP to bind (default 0.0.0.0)')
    p.add_argument('--port',       default=DEFAULT_PORT,      type=int, help='Port (default 9999)')
    p.add_argument('--marker-id',  default=DEFAULT_MARKER_ID, type=int, help='ArUco marker ID on HamBot')
    p.add_argument('--goal-x',     default=DEFAULT_GOAL_X,    type=float, help='Goal X position cm')
    p.add_argument('--goal-y',     default=DEFAULT_GOAL_Y,    type=float, help='Goal Y position cm')
    p.add_argument('--no-display', action='store_true',        help='Run headless, no OpenCV window')
    return p.parse_args()


if __name__ == '__main__':
    args = parse_args()

    print("=" * 60)
    print("World State Server")
    print("=" * 60)
    print(f"  Host      : {args.host}")
    print(f"  Port      : {args.port}")
    print(f"  Marker ID : {args.marker_id}")
    print(f"  Goal      : ({args.goal_x}, {args.goal_y}) cm")
    print(f"  Display   : {'no' if args.no_display else 'yes'}")
    print("=" * 60)

    server = WorldStateServer(
        host          = args.host,
        port          = args.port,
        marker_id     = args.marker_id,
        goal_position = (args.goal_x, args.goal_y),
        show_display  = not args.no_display,
    )
    server.run()
