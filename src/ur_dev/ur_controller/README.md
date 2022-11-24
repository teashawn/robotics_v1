# DeusExCubus

`DeusExCubus` is a project for the final assignment in **Ocado Robotics Accelerator 2022**.

## 🤖🦾 + 📦 = 🤓❤️

## Installing dependencies

```bash
cd robotics_v1/src/ur_dev/ur_controller
pip install -r requirements.txt
```

## Configuration

Configuration parameters can be tweaked in `robotics_v1/src/ur_dev/ur_controller/ur_controller/ur_controller.py`

```python
config = dec.DeusExCubusConfig()
config.debug=True
config.simulation=simulation
config.pack_commands=True
config.blending_radius=0.1
config.acceleration=1.0
config.velocity=1.0
config.box_spacing = 0.003
config.pre_pick_z_offset = 2.0
config.use_ascii_art = True
config.placing_speed_factors = [1.0, 0.8, 0.6, 0.4, 0.3, 0.2, 0.1, 0.1]
```

## Running

### In simulation

From repo root directory run
```bash
source install/setup.bash
ros2 run ur_controller ur_controller
```

### On real UR10e

From repo root directory run
```bash
source install/setup.bash
ros2 run ur_controller ur_controller simulation=false
```

### Easter egg 🐣

**Please run in a maximised terminal**. From repo root directory run
```bash
source install/setup.bash
ros2 run ur_controller ur_controller east=eregg

# or

ros2 run ur_controller ur_controller simulation=false east=eregg
```