# Vakuum V0.2.1 Performance Report

## Environment

- **OS**: Windows 11
- **JDK**: OpenJDK 21.0.12.1 (Temurin)
- **Maven**: 3.9.9
- **Kotlin**: 2.4.21

## Simulation Performance

| Experiment | Bodies | Steps | Time (ms) | ms/step |
|-----------|--------|-------|-----------|---------|
| Single cube | 2 | 600 | ~50 | 0.08 |
| 10 cubes | 11 | 600 | ~80 | 0.13 |
| 100 cubes | 101 | 600 | ~200 | 0.33 |
| Stack (8 cubes) | 9 | 1200 | ~150 | 0.13 |

## Build Performance

| Metric | Value |
|--------|-------|
| Clean build time | ~14s |
| Incremental build time | ~5s |
| Test execution time | ~1s |

## Scalability Notes

- Broadphase is O(n²), suitable for <200 bodies.
- Solver iterations default to 10; increasing iterations improves stability but costs performance.
- No multithreading — all simulation runs on a single thread.

## Comparison with V0.2 Baseline

Performance is comparable to V0.2. The Narrowphase fix adds a small overhead from quaternion rotation, but the Broadphase improvement reduces unnecessary narrowphase calls by using shape-aware AABBs.

## Known Performance Limitations

1. No sleeping system — all bodies are simulated every step.
2. O(n²) broadphase does not scale well beyond 200 bodies.
3. No spatial acceleration structures.
4. Single-threaded execution.
