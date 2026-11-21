# Vakuum V0.2 Test Report

## Test Summary

| Module | Tests | Passed | Failed |
|--------|-------|--------|--------|
| vakuum-math | 135 | 135 | 0 |
| vakuum-core | 40 | 40 | 0 |
| vakuum-dynamics | 41 | 41 | 0 |
| vakuum-collision | 15 | 15 | 0 |
| vakuum-solver | 10 | 10 | 0 |
| **Total** | **241** | **241** | **0** |

## Physics Test Results

### Gravity Integration
- Free-fall velocity after 1s: -9.81 m/s (expected: -9.81 m/s) ✅
- Free-fall position after 1s: 95.095 m (expected: 95.095 m) ✅
- Zero gravity maintains velocity ✅

### Collision Detection
- Box-box overlap detection ✅
- Box-plane penetration detection ✅
- Non-overlapping pairs return no contact ✅
- Edge cases: touching boxes ✅

### Collision Response
- Normal impulse reverses relative velocity ✅
- Friction reduces tangential velocity ✅
- Restitution causes bounce ✅
- Static body immobility ✅
- Penetration correction pushes bodies apart ✅

### Numerical Stability
- No NaN after 1000 integration steps ✅
- No NaN after 100 solver iterations ✅
- No NaN in 100-cube stress test (600 steps) ✅

### Reproducibility
- Two identical simulations produce byte-identical results ✅

## Experiment Results

### Experiment A — Single Falling Cube
- Cube starts at y=10.0, mass=1.0kg, restitution=0.3
- Final position: y≈0.59 (resting on ground)
- Final velocity: ≈0.04 m/s (near rest)
- No NaN detected ✅

### Experiment B — Multiple Cubes (10)
- 10 cubes at different positions and masses
- All cubes fall and settle without NaN ✅

### Experiment C — Cube Stacking (4×2)
- 8 cubes in a 4×2 stack
- Stack remains stable for 1200 steps ✅

### Experiment D — Stress Test (100 cubes)
- 100 cubes, 600 steps
- Time: 294ms (0.49ms per step)
- No NaN detected ✅

### Experiment E — Reproducibility
- Two identical runs produce identical results ✅

## Performance

| Metric | Value |
|--------|-------|
| Steps per second (100 cubes) | ~2,039 |
| Average step time (100 cubes) | 0.49ms |
| Memory | Single-threaded, no GC pressure |
