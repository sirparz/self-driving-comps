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
ACC_Development/ 
├── Development/ # Main working directory for ROS 2 packages and dev code 
│ └── ros2/ # Contains the ROS2 workspace
│ └── matlab-simulink/ 
├── docker/ 
│ └── Dockerfile.quanser # Docker setup for consistent dev environment 
├── [Other folders are ignored for privacy/sandbox use]

```
---

## 📦 Getting Started

### 1. Set up your system using Quanser's guide:
Follow: [ACC Software Setup Instructions](https://github.com/quanser/ACC-Competition-2025/blob/main/Software_Guides/ACC%20Software%20Setup%20Instructions.md)

### 2. Clone This Repository

After completing setup, clone this repository into your workspace directory:

```bash
cd ~/Documents
git clone https://github.com/sirparz/self-driving-comps ACC_Development
```


## 🧷 Additional Setup (Terminal Shortcuts)

Refer to [Development/additional_setup.md](Development/additional_setup.md) to configure your terminal so it can launch:
- Quanser Virtual Environment
- Isaac ROS Docker container
Directly from your desktop or with a shortcut.

## 📓 Development Guide (Isaac ROS)

Follow this official guide for:
- Container usage
- Workspace layout
- Persistent apt and pip3 installs in the Isaac ROS container

📘 [Quanser Development Guide](https://github.com/quanser/ACC-Competition-2025/blob/main/Software_Guides/Development%20Guide.md)
> 💡 Tip: To make apt or pip3 installs persistent inside the Isaac container, follow the instructions for modifying and committing the container as a custom image.

## ❓ FAQ

For common issues and troubleshooting:
📘 [Quanser ACC Competition FAQ](https://github.com/quanser/ACC-Competition-2025/blob/main/Software_Guides/FAQ.md)

## 🛡 License

This repository is private and all rights reserved.
Do not copy, distribute, or use this code without explicit permission.