# Vakuum Physics Engine — Benchmarks

## Overview

This document describes the benchmark suite for the Vakuum physics engine and provides guidance on running and interpreting benchmark results.

## Running Benchmarks

### Via Maven

Run all benchmarks as part of the test suite:

```bash
mvn test -pl vakuum-tests
```

Run the benchmark main class directly:

```bash
mvn -pl vakuum-tests exec:java -Dexec.mainClass="io.vakuum.benchmark.BenchmarkSuiteKt"
```

### Programmatic Usage

```kotlin
val suite = BenchmarkSuite()
val results = suite.runAll()

for (result in results) {
    println(result)
}
```

## Benchmark Scenarios

| # | Scenario | Bodies | Steps | Description |
|---|----------|--------|-------|-------------|
| 1 | 100 Bodies | 100 | 600 | Random positions, box shapes, gravity |
| 2 | 500 Bodies | 500 | 300 | Larger scale, box shapes, gravity |
| 3 | 1000 Bodies | 1000 | 200 | Maximum scale test, box shapes |
| 4 | Dense Collision | 100 | 300 | 100 bodies in a small area (4×4×10m) |
| 5 | Large Stack | 50 | 600 | 50 cubes stacked vertically |
| 6 | Mixed Shapes | 50 | 300 | Box, Sphere, Capsule, ConvexHull mix |
| 7 | CCD | 10 + 5 static | 300 | High-speed bodies with CCD enabled |
| 8 | Long Running | 100 | 1000 | Extended simulation stability |

## Measured Metrics

Each benchmark reports:

- **Avg Step Time**: Average simulation step duration in milliseconds
- **P50 Step Time**: Median step time (50th percentile)
- **P95 Step Time**: 95th percentile step time
- **P99 Step Time**: 99th percentile step time
- **Total Simulation Time**: Total time for all steps
- **NaN Detection**: Whether any body developed NaN values

## Hardware and JVM Configuration

Record your environment when running benchmarks:

```
Hardware: <OS name> <architecture>
CPU: <model and clock speed>
RAM: <total system memory>
JVM: <Java version> (<vendor>)
CPU Cores: <available processors>
Max Memory: <JVM max heap> MB
```

## Example Results

*The following are placeholder results. Run the benchmarks on your hardware for actual measurements.*

```
=== Summary ===
Benchmark                              Bodies    Avg(ms)    P95(ms)   Total(ms)
---------------------------------------------------------------------------
100 Bodies                                100     0.XXXX     0.XXXX      XXX.XX
500 Bodies                                500     0.XXXX     0.XXXX      XXX.XX
1000 Bodies                              1000     0.XXXX     0.XXXX      XXX.XX
Dense Collision (100 bodies)              100     0.XXXX     0.XXXX      XXX.XX
Large Stack (50 cubes)                    50      0.XXXX     0.XXXX      XXX.XX
Mixed Shapes (50 bodies)                  50      0.XXXX     0.XXXX      XXX.XX
CCD (10 fast bodies)                      10      0.XXXX     0.XXXX      XXX.XX
Long Running (100 bodies, 1000 steps)    100     0.XXXX     0.XXXX      XXX.XX
```

## Methodology

1. **Warmup**: Each benchmark runs 10 warmup steps before measurement to allow JIT compilation.
2. **Determinism**: All benchmarks use fixed random seeds for reproducible body placement.
3. **Isolation**: Each benchmark creates a fresh `SimulationPipeline` instance.
4. **No Sleeping**: Sleeping is disabled in benchmarks to ensure all bodies are processed each step.
5. **Fixed Timestep**: All benchmarks use 60 Hz fixed timestep (1/60s ≈ 16.67ms).

## Interpreting Results

- **Avg Step Time**: Lower is better. Indicates overall simulation throughput.
- **P95/P99**: Higher percentiles indicate occasional spikes. Large gaps between avg and P99 suggest inconsistent performance.
- **NaN Detection**: Must be `false`. Any NaN indicates numerical instability.
- **Total Time**: Wall-clock time for the entire benchmark. Useful for comparing overall simulation cost.

## Comparison Guidelines

When comparing benchmark results:

1. Use the same hardware and JVM version.
2. Run each benchmark at least 3 times and report the average.
3. Close other applications to minimize background interference.
4. Report the exact Vakuum commit hash for reproducibility.

## Known Limitations

- Benchmarks do not measure memory allocation rates (GC pressure).
- Broadphase/narrowphase/solver times are not measured separately.
- Single-threaded only; multithreading benchmarks are planned for future milestones.
