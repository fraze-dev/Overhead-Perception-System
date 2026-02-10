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

- **Mobile robots** (HamBot)
- **Colored balls** (objects of interest)
- **Static obstacles** and walls
- **Goal regions** or targets

The perception system transforms camera observations into a consistent world coordinate frame and provides real-time world-state information for autonomous robot navigation.

---

## ✨ Features

### Completed ✅
- [x] RealSense SDK integration and camera control
- [x] Depth and RGB stream capture with synchronization
- [x] Custom viewer with recording capabilities
- [x] Comprehensive depth accuracy testing suite
  - Distance accuracy: ±1-2.5 cm at 50-250 cm
  - Spatial uniformity analysis
  - Repeatability testing (0.36 cm std dev)
  - Environmental effects characterization
- [x] Camera characterization and validation (no calibration needed!)

### In Progress 🚧
- [x] World-frame coordinate transformation
- [ ] Extrinsic calibration methodology
- [ ] Object detection (color-based, depth-based)

### Planned 📅
- [ ] Robot and obstacle detection
- [ ] Multi-object tracking with temporal filtering
- [ ] Real-time visualization (top-down world map)
- [ ] Accuracy analysis across workspace
- [ ] Communication protocol (camera → robot)
- [ ] Robot behavior implementation using global state
- [ ] Full system integration and demonstration

---

## 🎯 Key Results (Week 2)

Our depth accuracy testing revealed excellent camera performance:

| Metric | Result | Target |
|--------|--------|--------|
| **Accuracy (50-250cm)** | <1.1% error | <2% spec |
| **Precision (2m)** | ±0.36 cm std dev | - |
| **Repeatability** | 2.1 cm range over 1000 frames | - |
| **Lighting Dependence** | Minimal (0.06 cm variation) | - |

**Key Finding:** Camera exceeds manufacturer specifications - no calibration needed! ✅

See [depth_accuracy_results_012426.md](depth_accuracy_results_012426.md) for full analysis.

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

### Running Tests

```bash
# Run depth accuracy test suite
python depth_accuracy_test_v2.py

# Results saved to: results/depth_accuracy/
```

---

## 📁 Project Structure

```
Overhead-Perception-System/
├── data/
│   ├── raw/              # Original camera data
│   ├── processed/        # Processed datasets
│   └── external/         # Third-party data
├── src/
│   ├── data/            # Data processing modules
│   ├── models/          # Detection/tracking models
│   └── utils/           # Helper functions
├── tests/               # Unit tests
├── results/             # Output figures and data
│   ├── figures/
│   └── depth_accuracy/  # Camera characterization results
├── notebooks/           # Jupyter notebooks for analysis
├── docs/                # Documentation
│   ├── Project_overview.md
│   ├── Project_timeline.md
│   └── depth_accuracy_results_012426.md
├── depth_accuracy_test_v2.py  # Camera testing script
├── requirements.txt
└── README.md
```

---

## 📊 Timeline

**16-Week Research Project (Jan 13 - May 2, 2026)**

| Phase | Weeks | Status | Deliverable |
|-------|-------|--------|-------------|
| Hardware Setup & RealSense API | 1-2 | ✅ Complete | Camera characterization |
| Coordinate Systems & Calibration | 3-4 | 🚧 In Progress | Calibration report |
| Object Detection | 5-6 | 📅 Planned | Detection prototype |
| Tracking & State Estimation | 7-8 | 📅 Planned | Live visualization |
| Accuracy Analysis | 9-10 | 📅 Planned | Mid-semester report |
| Communication Protocol | 11-12 | 📅 Planned | Protocol documentation |
| Robot Behavior & Integration | 13-14 | 📅 Planned | Full system demo |
| Analysis & Documentation | 15-16 | 📅 Planned | Final report & presentation |

See [Project_timeline.md](docs/Project_timeline.md) for detailed weekly breakdown.

---

## 🛠️ Technology Stack

- **Hardware:** Intel RealSense D435 depth camera
- **Language:** Python 3.13
- **Key Libraries:**
  - `pyrealsense2` - Camera SDK
  - `opencv-python` - Image processing
  - `numpy` - Numerical computation
  - `matplotlib` - Visualization
- **Development:** PyCharm, Git/GitHub

---

## 📈 Current Status (Week 4-5)

**Completed:**
- Hardware installation and SDK integration
- Comprehensive depth accuracy characterization
- Custom viewer with recording functionality
- Development environment and workflow established

**Next Steps:**
- Implement world-frame coordinate transformation
- Design extrinsic calibration approach
- Begin object detection exploration (color-based HSV masking)

---

## 📖 Documentation

- **[Project Overview](docs/Project_overview.md)** - Goals, components, and expected outcomes
- **[Project Timeline](docs/Project_timeline.md)** - Detailed 16-week schedule with milestones
- **[Depth Accuracy Results](docs/depth_accuracy_results_012426.md)** - Complete camera characterization report

---

## 🤝 Contributing

This is a research project under active development. Contributions, suggestions, and feedback are welcome!

### Project Team
- **Student Researcher:** Aaron Fraze ([@fraze-dev](https://github.com/fraze-dev))
- **Mentor:** Chance J. Hamilton

### Weekly Meetings
- **When:** Tuesdays, 2:00-3:00 PM
- **Focus:** Progress review, technical discussions, planning

---

## 🙏 Acknowledgments

- Intel RealSense SDK and community
- University of South Florida/Robotics Department
- Research mentor Chance J. Hamilton

---

## 📧 Contact

**Aaron Fraze**  
aaron.fraze2@gmail.com  
GitHub: [@fraze-dev](https://github.com/fraze-dev)

**Project Repository:** [github.com/fraze-dev/Overhead-Perception-System](https://github.com/fraze-dev/Overhead-Perception-System)

---

**Last Updated:** February 9, 2026
