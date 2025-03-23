# ACC Development

Private development workspace for the Quanser Autonomous Connected Car (ACC) Competition 2025.

---

## ⚙️ Prerequisites

Before using this repository, make sure your system is set up according to the official ACC software instructions:

👉 [Quanser ACC Software Setup Guide](https://github.com/quanser/ACC-Competition-2025/blob/main/Software_Guides/ACC%20Software%20Setup%20Instructions.md)

This includes installing:
- ROS 2 Humble
- Docker and NVIDIA Container Toolkit (for Jetson or GPU acceleration)
- Isaac ROS SDK (if applicable)
- Additional dependencies required for the ACC platform

---

## 🧭 Repository Structure
```
ACC_Development/ ├── Development/ # Main working directory for ROS 2 packages and dev code │ └── ros2/ # Contains src/, build/, install/, log/ ├── docker/ │ └── Dockerfile.quanser # Docker setup for consistent dev environment ├── [Other folders are ignored for privacy/sandbox use]

```
---

## 📦 Getting Started

### 1. Set up your system using Quanser's guide:
Follow: [ACC Software Setup Instructions](https://github.com/quanser/ACC-Competition-2025/blob/main/Software_Guides/ACC%20Software%20Setup%20Instructions.md)

### 2. Clone This Repository

After completing setup, clone this repository into your workspace directory:

```bash
mkdir -p ~/Documents
cd ~/Documents
git clone <your-private-repo-url> ACC_Development