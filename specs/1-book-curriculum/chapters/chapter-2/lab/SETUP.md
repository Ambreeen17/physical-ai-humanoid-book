# Chapter 2 Labs: Setup and Installation Guide

This guide provides step-by-step instructions for setting up the environment to run all Chapter 2 lab exercises.

## System Requirements

- **Operating System**: Ubuntu 22.04 LTS (recommended) or Ubuntu 20.04
- **Python**: 3.8 or higher
- **Memory**: 4GB RAM minimum (8GB recommended)
- **Storage**: 2GB free space

## Quick Start (Ubuntu 22.04)

### Option 1: Direct Installation

```bash
# Install Python dependencies
sudo apt-get update
sudo apt-get install -y python3 python3-pip python3-numpy python3-matplotlib python3-scipy

# Install MuJoCo
pip3 install mujoco==2.3.7

# Verify installation
python3 -c "import mujoco; print('MuJoCo version:', mujoco.__version__)"
```

### Option 2: Docker (Recommended)

```bash
# Build the Docker image
docker build -f Dockerfile.lab -t ros2-lab-ch2 .

# Run the container
docker run -it -v $(pwd):/lab ros2-lab-ch2
```

## Detailed Installation

### Step 1: Install Python and pip

```bash
sudo apt-get update
sudo apt-get install -y python3 python3-dev python3-pip
```

### Step 2: Install Required Python Packages

```bash
pip3 install --upgrade pip
pip3 install numpy matplotlib scipy
```

### Step 3: Install MuJoCo

```bash
# Option A: Using pip (recommended)
pip3 install mujoco==2.3.7

# Option B: From source (for development)
# 1. Download MuJoCo from https://mujoco.org/
# 2. Extract to ~/.mujoco/mujoco2137
# 3. Add to LD_LIBRARY_PATH:
#    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/.mujoco/mujoco2137/bin
```

### Step 4: Install Additional Dependencies (for visualization)

```bash
sudo apt-get install -y \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libosmesa6 \
    libxi6 \
    libxinerama1 \
    libxcursor1 \
    libxrandr2 \
    libxxf86vm1
```

### Step 5: Verify Installation

```bash
# Test MuJoCo
python3 -c "
import mujoco
import numpy as np

# Simple test
model = mujoco.MjModel.from_xml_string('''
<mujoco>
  <worldbody>
    <body name="test">
      <geom type="box" size="0.1 0.1 0.1"/>
    </body>
  </worldbody>
</mujoco>
''')
data = mujoco.MjData(model)
print('MuJoCo installation successful!')
"

# Test numpy and matplotlib
python3 -c "
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
print('NumPy version:', np.__version__)
print('All dependencies installed successfully!')
"
```

## Project Structure

```
lab/
├── Dockerfile.lab          # Docker environment
├── run_all_tests.sh        # Master test script
├── test_lab2_1.sh          # Lab 2.1 test
├── test_lab2_2.sh          # Lab 2.2 test
├── test_lab2_3.sh          # Lab 2.3 test
├── lab2_1/
│   ├── README.md
│   ├── lab2_1_starter.py
│   └── lab2_1_solution.py
├── lab2_2/
│   ├── README.md
│   ├── lab2_2_starter.py
│   └── lab2_2_solution.py
└── lab2_3/
    ├── README.md
    ├── lab2_3_starter.py
    ├── lab2_3_solution.py
    └── arm_model.xml
```

## Running the Labs

### Lab 2.1: Forward Kinematics (30 min)

```bash
cd lab2_1
python3 lab2_1_starter.py   # Run starter code with TODOs
python3 lab2_1_solution.py  # Run complete solution
```

### Lab 2.2: Inverse Kinematics (45 min)

```bash
cd lab2_2
python3 lab2_2_starter.py   # Run starter code with TODOs
python3 lab2_2_solution.py  # Run complete solution
```

### Lab 2.3: Dynamics Simulation (60 min)

```bash
cd lab2_3
python3 lab2_3_starter.py   # Run starter code with TODOs
python3 lab2_3_solution.py  # Run complete solution
```

## Troubleshooting

### Common Issues

#### 1. MuJoCo Not Found

**Error**: `ImportError: libmujoco.so: cannot open shared object file`

**Solution**:
```bash
# Add MuJoCo to library path
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/.mujoco/mujoco2137/bin

# Or install via pip (recommended)
pip3 install mujoco
```

#### 2. Matplotlib Display Error

**Error**: `RuntimeError: Invalid DISPLAY`

**Solution**:
```python
# Use non-interactive backend
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
```

#### 3. Permission Denied

**Error**: `Permission denied` when running scripts

**Solution**:
```bash
chmod +x *.sh
```

#### 4. ImportError for numpy/scipy

**Error**: `ModuleNotFoundError: No module named 'numpy'`

**Solution**:
```bash
pip3 install numpy scipy matplotlib
```

### Performance Issues

If simulations run slowly:

1. **Reduce simulation duration**: Edit `DURATION` parameter
2. **Increase timestep**: Change `DT` from 0.001 to 0.002
3. **Headless mode**: Use `matplotlib.use('Agg')` to skip rendering

### Verification Commands

```bash
# Check Python version
python3 --version

# Check installed packages
pip3 list | grep -E "numpy|mujoco|matplotlib|scipy"

# Test all labs
bash run_all_tests.sh
```

## Docker-Specific Instructions

### Building the Image

```bash
docker build -f Dockerfile.lab -t ros2-lab-ch2 .
```

### Running Individual Labs

```bash
# Lab 2.1
docker run -it -v $(pwd)/lab2_1:/lab/lab2_1 ros2-lab-ch2 bash
# Then run: python3 lab2_1_solution.py

# Lab 2.2
docker run -it -v $(pwd)/lab2_2:/lab/lab2_2 ros2-lab-ch2 bash
# Then run: python3 lab2_2_solution.py

# Lab 2.3
docker run -it -v $(pwd)/lab2_3:/lab/lab2_3 ros2-lab-ch2 bash
# Then run: python3 lab2_3_solution.py
```

### Running All Tests in Docker

```bash
docker run -it -v $(pwd):/lab ros2-lab-ch2 bash
# Then run: bash run_all_tests.sh
```

## Additional Resources

- **MuJoCo Documentation**: https://mujoco.readthedocs.io/
- **NumPy Documentation**: https://numpy.org/doc/
- **Matplotlib Documentation**: https://matplotlib.org/stable/
- **ROS 2 Humble**: https://docs.ros.org/en/humble/

## Support

If you encounter issues not covered here:

1. Check the individual lab README files for troubleshooting
2. Review error messages carefully
3. Verify all dependencies are installed
4. Ensure you're using the correct Python version (3.8+)
