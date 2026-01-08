# Verified Autonomy

## Prerequisites

- Linux host (tested on Ubuntu)
- ~10GB disk space (Docker image + Unity environments)

## Quick Start

### 1. Clone this repository

```bash
git clone https://github.com/gavin-hyl/verified_autonomy.git
cd verified_autonomy
```

### 2. Clone the Vector Navigation Stack

```bash
git clone https://github.com/VectorRobotics/vector_navigation_stack.git
```

### 3. Install Docker (if not installed)

```bash
sudo apt update
sudo apt install -y docker.io docker-compose-v2
sudo usermod -aG docker $USER
```

### 4. Follow the README in `docker/`

## Links

- [Vector Navigation Stack](https://github.com/VectorRobotics/vector_navigation_stack)
- [CMU Exploration Development Environment](https://www.cmu-exploration.com/development-environment)
- [Unity Environments (Google Drive)](https://drive.google.com/drive/folders/1G1JYkccvoSlxyySuTlPfvmrWoJUO8oSs?usp=sharing)
