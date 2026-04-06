#!/bin/bash

# Setup script for Gazebo RL Training dependencies

set -e

echo "======================================"
echo "Gazebo RL Training - Dependency Setup"
echo "======================================"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# echo -e "${GREEN}[1/5] Installing Python dependencies...${NC}"
# pip3 install --upgrade pip

# # Core ML/RL packages
# echo "Installing stable-baselines3..."
# pip3 install stable-baselines3>=1.8.0

# echo "Installing gym (for RL environment interface)..."
# pip3 install gym>=0.21.0

# echo "Installing TensorFlow/PyTorch (choose one)..."
# pip3 install torch>=1.10.0  # For PyTorch-based algorithms

# echo "Installing monitoring tools..."
# pip3 install tensorboard>=2.8.0
# pip3 install opencv-python>=4.5.0

# echo -e "${GREEN}[2/5] Verifying ROS packages...${NC}"
# # Check for rotors_simulator
# if ! dpkg -l | grep -q "ros.*rotors"; then
#     echo -e "${YELLOW}Installing rotors_simulator...${NC}"
#     # Note: This might need to be built from source, add to catkin_ws instead
#     echo "Add to ~/catkin_ws/src if not already present"
# else
#     echo "rotors_simulator already installed"
# fi

# echo -e "${GREEN}[3/5] Building ROS packages...${NC}"
# if [ -d ~/catkin_ws ]; then
#     cd ~/catkin_ws
#     echo "Building workspace..."
#     catkin build -j4 2>&1 | tail -20
# else
#     echo -e "${YELLOW}catkin_ws not found at ~/catkin_ws${NC}"
# fi

echo -e "${GREEN}[4/5] Verifying installation...${NC}"
echo "Checking Python packages..."
python3 -c "import stable_baselines3; print('✓ stable-baselines3:', stable_baselines3.__version__)"
python3 -c "import gymnasium; print('✓ gymnasium:', gymnasium.__version__)"
python3 -c "import torch; print('✓ torch:', torch.__version__)"
python3 -c "import cv2; print('✓ opencv:', cv2.__version__)"

echo -e "${GREEN}[5/5] Creating directory structure...${NC}"
mkdir -p ~/catkin_ws/src/flightmare/viri/{models,logs,data}
echo "Directories created:"
echo "  - ~/catkin_ws/src/flightmare/viri/models/ (for trained models)"
echo "  - ~/catkin_ws/src/flightmare/viri/logs/ (for TensorBoard logs)"
echo "  - ~/catkin_ws/src/flightmare/viri/data/ (for training data)"

echo ""
echo -e "${GREEN}======================================"
echo "Setup complete!"
echo "======================================${NC}"
echo ""
echo "Next steps:"
echo "1. Make launch scripts executable:"
echo "   chmod +x ~/catkin_ws/src/flightmare/viri/scripts/*.sh"
echo ""
echo "2. Source ROS setup:"
echo "   source ~/catkin_ws/devel/setup.bash"
echo ""
echo "3. Start training:"
echo "   roslaunch viri gazebo_rl_training.launch"
echo "   # In another terminal:"
echo "   python3 scripts/train_sb3_gazebo.py --train --algo PPO"
echo ""
echo "4. Monitor training:"
echo "   tensorboard --logdir=./logs/"
echo ""
