"""
Vakuum Simulation Validator
===========================
Validates exported JSON simulation data without requiring Blender.
Checks:
  1. JSON schema validity
  2. Free-fall physics (v = g*t)
  3. Ground collision detection
  4. Bounce behavior
  5. Final resting state
  6. No NaN/Infinity values
  7. Deterministic timestamps

Usage:
  python validate_simulation.py output/experiment_a_single_cube.json
"""
import json
import sys
import math

EPSILON = 1e-6

def load_simulation(path):
    """Load and validate JSON schema."""
    with open(path, 'r') as f:
        data = json.load(f)
    
    if 'metadata' not in data:
        raise ValueError("Missing 'metadata' in JSON")
    if 'frames' not in data:
        raise ValueError("Missing 'frames' in JSON")
    
    meta = data['metadata']
    required_meta = ['timestep', 'gravity', 'frameCount']
    for key in required_meta:
        if key not in meta:
            raise ValueError(f"Missing metadata key: {key}")
    
    return data

def check_no_nan_inf(data):
    """Check that no body has NaN or Infinity values."""
    errors = []
    for frame_idx, frame in enumerate(data['frames']):
        for body in frame['bodies']:
            for field in ['position', 'orientation', 'linVel', 'angVel']:
                for val_idx, val in enumerate(body[field]):
                    if math.isnan(val):
                        errors.append(f"Frame {frame_idx}, body {body['id']}, {field}[{val_idx}] is NaN")
                    if math.isinf(val):
                        errors.append(f"Frame {frame_idx}, body {body['id']}, {field}[{val_idx}] is Infinity")
    return errors

def check_free_fall(data):
    """
    Verify that the dynamic body falls under gravity.
    Expected: v_y ≈ g * t (negative, since gravity is downward)
    """
    meta = data['metadata']
    g = meta['gravity'][1]  # -9.81
    dt = meta['timestep']
    
    # Find the dynamic body (id != 0)
    dynamic_body_id = None
    for body in data['frames'][0]['bodies']:
        if body['id'] != 0:
            dynamic_body_id = body['id']
            break
    
    if dynamic_body_id is None:
        return ["No dynamic body found"]
    
    errors = []
    # Check first few frames (before collision)
    for frame in data['frames'][:30]:
        for body in frame['bodies']:
            if body['id'] != dynamic_body_id:
                continue
            
            t = frame['time']
            expected_vy = g * t
            actual_vy = body['linVel'][1]
            
            # Allow some tolerance for integration error
            if abs(actual_vy - expected_vy) > 0.5:
                errors.append(
                    f"Frame {frame['frame']}: expected vy={expected_vy:.4f}, "
                    f"actual vy={actual_vy:.4f} (diff={abs(actual_vy - expected_vy):.4f})"
                )
    
    return errors

def check_ground_collision(data):
    """
    Verify that the cube collides with the ground.
    The cube should not pass through y=0 (ground level).
    """
    meta = data['metadata']
    errors = []
    
    for frame in data['frames']:
        for body in frame['bodies']:
            if body['id'] == 0:
                continue  # skip ground
            
            y = body['position'][1]
            # Cube half-extent is 0.5, so bottom of cube is at y - 0.5
            # Ground is at y=0, so cube bottom should not go below 0
            cube_bottom = y - 0.5
            if cube_bottom < -0.1:  # Allow small penetration
                errors.append(
                    f"Frame {frame['frame']}: cube bottom at y={cube_bottom:.4f}, "
                    f"passed through ground!"
                )
    
    return errors

def check_bounce(data):
    """
    Verify that the cube bounces after hitting the ground.
    After the first collision, velocity should reverse direction briefly.
    """
    errors = []
    prev_vy = 0.0
    bounced = False
    
    for frame in data['frames']:
        for body in frame['bodies']:
            if body['id'] == 0:
                continue
            
            vy = body['linVel'][1]
            
            # Check if velocity changed from negative to positive (bounce)
            if prev_vy < -1.0 and vy > 0.0:
                bounced = True
            
            prev_vy = vy
    
    if not bounced:
        errors.append("No bounce detected — cube may have passed through ground or stopped without bouncing")
    
    return errors

def check_resting_state(data):
    """
    Verify that the cube eventually comes to rest on the ground.
    Last 50 frames should have near-zero velocity.
    """
    errors = []
    last_frames = data['frames'][-50:]
    
    for frame in last_frames:
        for body in frame['bodies']:
            if body['id'] == 0:
                continue
            
            speed = math.sqrt(
                body['linVel'][0]**2 + body['linVel'][1]**2 + body['linVel'][2]**2
            )
            
            if speed > 0.5:
                errors.append(
                    f"Frame {frame['frame']}: cube still moving at speed={speed:.4f} m/s "
                    f"in final frames"
                )
    
    # Check final position
    last_frame = data['frames'][-1]
    for body in last_frame['bodies']:
        if body['id'] == 0:
            continue
        
        y = body['position'][1]
        # Cube half-extent is 0.5, so center should be around y=0.5
        if abs(y - 0.5) > 0.3:
            errors.append(
                f"Final cube y={y:.4f}, expected ~0.5 (cube resting on ground)"
            )
    
    return errors

def check_timestamps(data):
    """Verify that timestamps are consistent with the timestep."""
    meta = data['metadata']
    dt = meta['timestep']
    errors = []
    
    for frame in data['frames']:
        expected_time = frame['frame'] * dt
        actual_time = frame['time']
        if abs(actual_time - expected_time) > 1e-6:
            errors.append(
                f"Frame {frame['frame']}: expected time={expected_time:.10f}, "
                f"actual={actual_time:.10f}"
            )
    
    return errors

def check_quaternion_normalization(data):
    """Verify that all orientation quaternions are unit length."""
    errors = []
    for frame_idx, frame in enumerate(data['frames']):
        for body in frame['bodies']:
            q = body['orientation']
            length = math.sqrt(q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2)
            if abs(length - 1.0) > 0.01:
                errors.append(
                    f"Frame {frame_idx}, body {body['id']}: quaternion length={length:.6f}"
                )
    return errors

def main():
    if len(sys.argv) < 2:
        print("Usage: python validate_simulation.py <json_file>")
        print("Example: python validate_simulation.py output/experiment_a_single_cube.json")
        sys.exit(1)
    
    json_path = sys.argv[1]
    print(f"Validating simulation: {json_path}")
    print("=" * 60)
    
    # Load
    try:
        data = load_simulation(json_path)
        print(f"[OK] JSON schema valid")
        print(f"     Frames: {len(data['frames'])}")
        print(f"     Timestep: {data['metadata']['timestep']:.6f}s")
        print(f"     Gravity: {data['metadata']['gravity']}")
    except Exception as e:
        print(f"[FAIL] JSON load error: {e}")
        sys.exit(1)
    
    all_passed = True
    
    checks = [
        ("No NaN/Infinity", check_no_nan_inf),
        ("Free-fall physics", check_free_fall),
        ("Ground collision", check_ground_collision),
        ("Bounce detection", check_bounce),
        ("Resting state", check_resting_state),
        ("Timestamps", check_timestamps),
        ("Quaternion normalization", check_quaternion_normalization),
    ]
    
    for name, check_fn in checks:
        errors = check_fn(data)
        if errors:
            print(f"[FAIL] {name}")
            for err in errors[:5]:  # Show first 5 errors
                print(f"       {err}")
            if len(errors) > 5:
                print(f"       ... and {len(errors) - 5} more")
            all_passed = False
        else:
            print(f"[OK]   {name}")
    
    print("=" * 60)
    if all_passed:
        print("RESULT: All checks PASSED ✓")
        sys.exit(0)
    else:
        print("RESULT: Some checks FAILED ✗")
        sys.exit(1)

if __name__ == "__main__":
    main()
