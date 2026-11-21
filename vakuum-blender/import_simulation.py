import bpy
import json
import os

# Path to the JSON simulation file exported by Vakuum
json_path = "output/experiment_a_single_cube.json"

def import_simulation(json_path):
    # Clear existing scene
    bpy.ops.wm.read_factory_settings(use_empty=True)
    
    # Load JSON data
    with open(json_path, 'r') as f:
        data = json.load(f)
    
    metadata = data['metadata']
    frames = data['frames']
    
    print(f"Importing {len(frames)} frames from Vakuum simulation")
    print(f"Timestep: {metadata['timestep']}")
    print(f"Gravity: {metadata['gravity']}")
    
    # Set frame range
    bpy.context.scene.frame_start = 0
    bpy.context.scene.frame_end = len(frames) - 1
    bpy.context.scene.render.fps = int(1.0 / metadata['timestep'])
    
    # Create objects from first frame
    bodies = {}
    first_frame = frames[0]
    
    for body_data in first_frame['bodies']:
        body_id = body_data['id']
        pos = body_data['position']
        
        # Create mesh based on body type
        if body_id == 0:
            # Ground plane
            bpy.ops.mesh.primitive_plane_add(size=20, location=(pos[0], pos[1], pos[2]))
            obj = bpy.context.active_object
            obj.name = f"ground_{body_id}"
            obj.scale = (5, 5, 1)
        else:
            # Dynamic cube
            bpy.ops.mesh.primitive_cube_add(size=1, location=(pos[0], pos[1], pos[2]))
            obj = bpy.context.active_object
            obj.name = f"cube_{body_id}"
        
        bodies[body_id] = obj
    
    # Animate each frame
    for frame_data in frames:
        frame_num = frame_data['frame']
        bpy.context.scene.frame_set(frame_num)
        
        for body_data in frame_data['bodies']:
            body_id = body_data['id']
            obj = bodies.get(body_id)
            if obj is None:
                continue
            
            pos = body_data['position']
            quat = body_data['orientation']  # [w, x, y, z]
            
            # Set location
            obj.location[0] = pos[0]
            obj.location[1] = pos[1]
            obj.location[2] = pos[2]
            
            # Set rotation (Blender uses w, x, y, z order)
            obj.rotation_quaternion[0] = quat[0]  # w
            obj.rotation_quaternion[1] = quat[1]  # x
            obj.rotation_quaternion[2] = quat[2]  # y
            obj.rotation_quaternion[3] = quat[3]  # z
            
            # Insert keyframes
            obj.keyframe_insert(data_path="location", index=-1)
            obj.keyframe_insert(data_path="rotation_quaternion", index=-1)
    
    print(f"Import complete. {len(bodies)} bodies animated over {len(frames)} frames.")
    return bodies

if __name__ == "__main__":
    if os.path.exists(json_path):
        import_simulation(json_path)
    else:
        print(f"JSON file not found: {json_path}")
        print("Run a simulation first:")
        print("  mvn exec:java -pl vakuum-runner -Dexec.mainClass=io.vakuum.runner.SimulationRunnerKt -Dexec.args=A output")
