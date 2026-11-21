# Vakuum Blender Bridge

This directory contains Python scripts for importing Vakuum simulation data into Blender.

## Usage

1. Run a simulation from the command line:
   ```
   mvn exec:java -pl vakuum-runner -Dexec.mainClass="io.vakuum.runner.SimulationRunnerKt" -Dexec.args="A output"
   ```

2. Open Blender and run `import_simulation.py` from the Scripting workspace:
   - Text → Open → `vakuum-blender/import_simulation.py`
   - Edit the `json_path` variable to point to your JSON file
   - Run Script

3. Optionally run `render_experiment.py` to set up camera and lighting.

## Coordinate System

- Vakuum uses Y-up, right-handed coordinate system
- Blender uses Y-up by default — no conversion needed
- Quaternions are exported as (w, x, y, z) — Blender uses the same order

## Important

- Never add Blender Rigid Body components to the imported cubes
- Never use Blender's physics simulation
- All motion is calculated by Vakuum and imported as keyframes
