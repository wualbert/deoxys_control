# End-Effector Coordinate Frame Visualization

## Overview
This feature adds real-time visualization of the end-effector coordinate frame in the PyBullet visualizer. The coordinate frame is automatically displayed at link index 8 (the end-effector link) whenever the robot is visualized.

## Features
- **Automatic Frame Display**: The coordinate frame is automatically drawn when `use_visualizer=True`
- **Real-time Updates**: The frame position and orientation update as the robot moves
- **Color-coded Axes**:
  - 🔴 RED = X-axis
  - 🟢 GREEN = Y-axis
  - 🔵 BLUE = Z-axis
- **No Clutter**: Old frames are automatically removed before drawing new ones

## Implementation Details

### Files Modified

1. **`deoxys/franka_interface/visualizer.py`**:
   - Added `draw_coordinate_frame()` method to draw axes at any pose
   - Added `draw_ee_coordinate_frame()` method to draw frame at end-effector
   - Modified `update()` to automatically draw EE frame when robot state updates
   - Stores frame line IDs to prevent accumulation

2. **`deoxys/franka_interface/franka_interface.py`**:
   - Fixed initialization order to create visualizer before mock state setup
   - Removed duplicate visualizer initialization

### How It Works

1. When `FrankaInterface` is initialized with `use_visualizer=True`:
   - A `PybulletVisualizer` instance is created
   - The visualizer loads the robot model in a GUI PyBullet instance

2. When the robot state is updated (via `visualizer.update()`):
   - The visualizer gets the end-effector pose using `getLinkState()`
   - It draws three colored lines representing the X, Y, and Z axes
   - Old frame lines are removed to keep the display clean

3. The frame correctly reflects the pose of link 8 in the visualizer's robot model

## Usage

### Basic Example
```python
from deoxys.franka_interface import FrankaInterface

# Initialize with visualization
robot = FrankaInterface(
    general_cfg_file="deoxys/config/local-host.yml",
    mock_mode=True,
    use_visualizer=True
)

# The coordinate frame will automatically appear at the end-effector
# and update as the robot moves
```

### Running the Demo
```bash
# Simple static visualization
PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python python test_ee_frame_simple.py

# Dynamic visualization with motion
PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python python demo_ee_frame_viz.py

# Full test with circular motion
PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python python test_ee_frame_visualization.py
```

## Configuration

### Customizing Frame Appearance
The frame appearance can be customized by modifying the `draw_ee_coordinate_frame()` parameters:

```python
# In visualizer.py, modify the default parameters:
def draw_ee_coordinate_frame(self, axis_length=0.1, line_width=3):
    # axis_length: Length of each axis line (meters)
    # line_width: Width of the lines (pixels)
```

### Disabling the Frame
To disable the frame while keeping the visualizer:

```python
# In the update() method call:
visualizer.update(joint_positions, draw_ee_frame=False)
```

## Technical Notes

1. **Coordinate System**: The frame represents the end-effector coordinate system in world coordinates
2. **Link Index**: The frame is drawn at link index 8, which is the standard end-effector link for the Franka Panda
3. **Performance**: The frame visualization has minimal performance impact as it only updates when the robot state changes
4. **Multiple Instances**: The implementation properly handles the PyBullet instance used by the visualizer to ensure correct frame placement

## Troubleshooting

- **Frame not visible**: Ensure `use_visualizer=True` when creating `FrankaInterface`
- **Frame at wrong location**: Make sure you're using the latest code with the fix that uses the visualizer's robot instance
- **Too many lines**: Old implementation issue - update to the latest code that removes old lines

## Future Enhancements

Potential improvements could include:
- Multiple coordinate frames (base, joints, etc.)
- Configurable colors and styles
- Text labels for axes
- Optional grid or reference frames
- Trajectory visualization