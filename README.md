# Overhead Perception System
**Global Tracking and World-State Estimation Using Intel RealSense D435**

[![Python 3.13](https://img.shields.io/badge/python-3.13-blue.svg)](https://www.python.org/downloads/)
[![RealSense SDK](https://img.shields.io/badge/RealSense-2.57.5-orange.svg)](https://github.com/IntelRealSense/librealsense)

A real-time overhead perception system that tracks robots, objects, and obstacles in world coordinates using an Intel RealSense depth camera. Developed as a research project at University of South Florida.

**Researcher:** Aaron Fraze
**Mentor:** Chance J. Hamilton
**Semester:** Spring 2026

---

## 📋 Project Overview

This system uses an overhead-mounted Intel RealSense D435 depth camera to create a "global coordinator" view of a robotics arena. It tracks:

- **Mobile robots** (HamBot) — via ArUco marker (primary) or HSV color fallback
- **Colored balls** — via HSV color segmentation
- **Static obstacles** and walls
- **Goal regions** or targets

The perception system transforms camera observations into a consistent world coordinate frame and streams real-time world-state over TCP to the robot.

---

## ✨ Features

### Completed ✅
- [x] RealSense SDK integration and camera control
- [x] Depth and RGB stream capture with synchronization
- [x] World-frame coordinate transformation (calibrated)
- [x] ArUco marker detection — robot pose + heading
- [x] HSV color segmentation — ball detection + robot fallback
- [x] Unified world-state estimator (ArUco primary, HSV fallback)
- [x] Bounding box visualization overlay
- [x] TCP server — streams world-state JSON to robot at ~30 FPS
- [x] HamBot receiver — robot-side TCP client with behavior logic
- [x] Detection benchmark suite with performance comparison

### In Progress 🚧
- [ ] Multi-object tracking with temporal filtering
- [ ] Obstacle/wall detection
- [ ] Robot navigation behavior (ball-pushing task)

### Planned 📅
- [ ] Full system integration and demonstration
- [ ] Accuracy analysis across full workspace
- [ ] Final report and presentation

---

## 🎯 Key Results

### Calibration (Week 4)
| Metric | Result |
|--------|--------|
| Center workspace error | < 5 cm |
| Edge workspace error | 6–7 cm |
| Coverage area | 3.3 m² |
| Z-axis systematic offset | −4 to −6 cm (correctable) |

### Full Pipeline Performance
| Metric | Result | Target |
|--------|--------|--------|
| End-to-end FPS (detect → TCP → robot decision) | ~28 FPS | ≥25 FPS |

The full pipeline includes camera capture, ArUco + HSV detection, world-state serialization, TCP transmission, and robot-side decision making.

---

## 🗂️ Project Structure

```
Overhead-Perception-System/
├── src/
│   ├── camera.py                  # RealSense camera wrapper, world-frame transform
│   ├── world_state.py             # Unified detector integration (ArUco + HSV)
│   ├── world_state_server.py      # TCP server — streams state to robot
│   ├── aruco_detector.py          # ArUco marker detection (robot primary)
│   ├── hsv_detector.py            # HSV color segmentation (ball + robot fallback)
│   ├── hsv_profiles.json          # Saved HSV tuning profiles
│   ├── depth_segmenter.py         # Depth-based object segmentation
│   ├── hambot_receiver.py         # Robot-side TCP client and behavior logic
│   ├── detection_benchmark.py     # Performance benchmarking tool
│   ├── overhead_perceptor_v1.py   # Early perception prototype (reference)
│   └── archive/                   # Older exploration scripts
├── results/
│   └── calibration/
│       └── calibration.json       # Camera extrinsic calibration parameters
├── docs/
│   ├── Project_overview.md
│   ├── Project_timeline.md
│   └── Calibration_Report_Final.md
├── requirements.txt
└── README.md
```

---

## 🚀 Quick Start

### Prerequisites
- Python 3.13+
- Intel RealSense SDK 2.57.5
- Intel RealSense D435 camera

### Installation

```bash
# Clone repository
git clone https://github.com/fraze-dev/Overhead-Perception-System.git
cd Overhead-Perception-System

# Create virtual environment
python -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

### Running the Perception Server (Overhead PC)

```bash
# Start world-state TCP server (streams to robot)
python src/world_state_server.py

# Optional arguments:
#   --host    IP to bind (default: 0.0.0.0)
#   --port    Port number (default: 9999)
```

### Running the Robot Receiver (HamBot)

```bash
# Start receiver on the robot
python src/hambot_receiver.py --host <overhead-pc-ip> --port 9999
```

### Running Detection Benchmarks

```bash
python src/detection_benchmark.py
# Results saved as JSON + Markdown in src/
```

---

## 🔌 System Architecture

```
┌─────────────────────────────────┐         ┌──────────────────────┐
│         Overhead PC             │   TCP   │       HamBot         │
│                                 │ ──────► │                      │
│  RealSense D435                 │  JSON   │  hambot_receiver.py  │
│       ↓                         │  ~30Hz  │                      │
│  camera.py                      │         │  - Parse world state │
│       ↓                         │         │  - Make decisions    │
│  world_state.py                 │         │  - Drive motors      │
│  (ArUco + HSV detectors)        │         └──────────────────────┘
│       ↓                         │
│  world_state_server.py          │
└─────────────────────────────────┘
```

**Message format:** JSON over TCP, one message per frame (~30/sec)
```json
{
  "timestamp": 1234567890.123,
  "robot": { "x": 45.2, "y": 30.1, "heading": 1.57, "heading_current": true },
  "ball":  { "x": 80.0, "y": 60.0, "vx": 0.0, "vy": 0.0 },
  "goal":  { "x": 110.0, "y": 0.0 }
}
```

---

## 🛠️ Technology Stack

- **Hardware:** Intel RealSense D435 depth camera
- **Language:** Python 3.13
- **Key Libraries:**
  - `pyrealsense2` — Camera SDK
  - `opencv-python` — Image processing, ArUco detection
  - `numpy` — Numerical computation
  - `matplotlib` — Visualization
  - `scipy` — Signal processing / filtering
- **Communication:** TCP (JSON over socket)
- **Development:** PyCharm, Git/GitHub

---

## 📊 Timeline

**16-Week Research Project (Jan 13 – May 2, 2026)**

| Phase | Weeks | Status | Deliverable |
|-------|-------|--------|-------------|
| Hardware Setup & RealSense API | 1–2 | ✅ Complete | Camera characterization |
| Coordinate Systems & Calibration | 3–4 | ✅ Complete | Calibration report |
| Object Detection | 5–6 | ✅ Complete | Detection benchmark |
| Tracking & State Estimation | 7–8 | ✅ Complete | World-state server + HamBot receiver |
| Accuracy Analysis | 9–10 | 🚧 In Progress | Mid-semester report |
| Robot Behavior & Integration | 11–13 | 📅 Planned | Full system demo |
| Analysis & Documentation | 14–16 | 📅 Planned | Final report & presentation |

See [Project_timeline.md](docs/Project_timeline.md) for detailed weekly breakdown.

---

## 📖 Documentation

- **[Project Overview](docs/Project_overview.md)** — Goals, components, and expected outcomes
- **[Project Timeline](docs/Project_timeline.md)** — Detailed 16-week schedule
- **[Calibration Report](docs/Calibration_Report_Final.md)** — World-frame calibration methodology and accuracy

---

## 🤝 Project Team

- **Student Researcher:** Aaron Fraze ([@fraze-dev](https://github.com/fraze-dev))
- **Mentor:** Chance J. Hamilton
- **Institution:** University of South Florida
- **Weekly Meetings:** Tuesdays, 2:00–3:00 PM

---

## 🙏 Acknowledgments

- Intel RealSense SDK and community
- University of South Florida Robotics Department
- Research mentor Chance J. Hamilton

---

## 📧 Contact

**Aaron Fraze**
aaron.fraze2@gmail.com
GitHub: [@fraze-dev](https://github.com/fraze-dev)

**Project Repository:** [github.com/fraze-dev/Overhead-Perception-System](https://github.com/fraze-dev/Overhead-Perception-System)

---

**Last Updated:** March 25, 2026
