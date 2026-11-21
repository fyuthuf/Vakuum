package io.vakuum.benchmark

import io.vakuum.collision.BoxShape
import io.vakuum.collision.CapsuleShape
import io.vakuum.collision.ConvexHullShape
import io.vakuum.collision.SphereShape
import io.vakuum.core.PhysicsConfiguration
import io.vakuum.dynamics.BodyType
import io.vakuum.dynamics.rigidBody
import io.vakuum.math.Vector3
import io.vakuum.runner.SimulationPipeline
import kotlin.math.abs
import kotlin.math.sqrt

/**
 * Benchmark suite for Vakuum physics engine.
 *
 * Measures simulation step time, P50/P95/P99 latency, and total simulation time
 * for various scenarios: 100/500/1000 bodies, dense collisions, stacks,
 * mixed shapes, CCD, and long-running simulations.
 *
 * All benchmarks are deterministic with fixed random seeds.
 */
class BenchmarkSuite {

    data class BenchmarkResult(
        val name: String,
        val bodyCount: Int,
        val stepCount: Int,
        val avgStepTimeMs: Double,
        val p50StepTimeMs: Double,
        val p95StepTimeMs: Double,
        val p99StepTimeMs: Double,
        val totalSimulationTimeMs: Double,
        val hasNaN: Boolean
    ) {
        override fun toString(): String {
            return """
                Benchmark: $name
                Bodies: $bodyCount
                Steps: $stepCount
                Avg Step: ${"%.4f".format(avgStepTimeMs)} ms
                P50: ${"%.4f".format(p50StepTimeMs)} ms
                P95: ${"%.4f".format(p95StepTimeMs)} ms
                P99: ${"%.4f".format(p99StepTimeMs)} ms
                Total: ${"%.2f".format(totalSimulationTimeMs)} ms
                NaN: $hasNaN
            """.trimIndent()
        }
    }

    /**
     * Simple deterministic random number generator for reproducibility.
     */
    class SeededRandom(var seed: Long) {
        fun nextDouble(): Double {
            seed = (seed * 1103515245L + 12345L) and 0x7FFFFFFFL
            return seed.toDouble() / 0x7FFFFFFF.toDouble()
        }

        fun nextDouble(min: Double, max: Double): Double {
            return min + nextDouble() * (max - min)
        }
    }

    private fun runBenchmark(
        name: String,
        bodyCount: Int,
        stepCount: Int,
        config: PhysicsConfiguration,
        setup: (SimulationPipeline) -> Unit
    ): BenchmarkResult {
        val pipeline = SimulationPipeline(config)
        setup(pipeline)

        // Warmup: 10 steps to JIT compile
        repeat(10) {
            pipeline.step()
        }

        // Measure
        val stepTimes = mutableListOf<Double>()
        val startNanos = System.nanoTime()

        repeat(stepCount) {
            val stepStart = System.nanoTime()
            pipeline.step()
            val stepEnd = System.nanoTime()
            stepTimes.add((stepEnd - stepStart) / 1_000_000.0) // ms
        }

        val endNanos = System.nanoTime()
        val totalMs = (endNanos - startNanos) / 1_000_000.0

        stepTimes.sort()
        val avg = stepTimes.average()
        val p50 = stepTimes[(stepTimes.size * 0.5).toInt().coerceIn(0, stepTimes.size - 1)]
        val p95 = stepTimes[(stepTimes.size * 0.95).toInt().coerceIn(0, stepTimes.size - 1)]
        val p99 = stepTimes[(stepTimes.size * 0.99).toInt().coerceIn(0, stepTimes.size - 1)]

        return BenchmarkResult(
            name = name,
            bodyCount = bodyCount,
            stepCount = stepCount,
            avgStepTimeMs = avg,
            p50StepTimeMs = p50,
            p95StepTimeMs = p95,
            p99StepTimeMs = p99,
            totalSimulationTimeMs = totalMs,
            hasNaN = pipeline.hasNaN()
        )
    }

    /**
     * Benchmark 1: 100 rigid bodies falling under gravity.
     */
    fun benchmark100Bodies(): BenchmarkResult {
        val config = PhysicsConfiguration(
            gravity = Vector3(0.0, -9.81, 0.0),
            fixedTimestep = 1.0 / 60.0,
            maxSolverIterations = 10,
            linearDamping = 0.01,
            angularDamping = 0.01,
            allowSleeping = false
        )

        return runBenchmark("100 Bodies", 100, 600, config) { pipeline ->
            val random = SeededRandom(42)
            for (i in 0 until 100) {
                val body = rigidBody()
                    .id(i)
                    .bodyType(BodyType.DYNAMIC)
                    .position(Vector3(
                        random.nextDouble(-10.0, 10.0),
                        random.nextDouble(5.0, 30.0),
                        random.nextDouble(-10.0, 10.0)
                    ))
                    .mass(random.nextDouble(0.5, 5.0))
                    .build()
                pipeline.addBody(body)
                pipeline.setShape(i, BoxShape(Vector3(0.5, 0.5, 0.5)))
            }
        }
    }

    /**
     * Benchmark 2: 500 rigid bodies.
     */
    fun benchmark500Bodies(): BenchmarkResult {
        val config = PhysicsConfiguration(
            gravity = Vector3(0.0, -9.81, 0.0),
            fixedTimestep = 1.0 / 60.0,
            maxSolverIterations = 10,
            linearDamping = 0.01,
            angularDamping = 0.01,
            allowSleeping = false
        )

        return runBenchmark("500 Bodies", 500, 300, config) { pipeline ->
            val random = SeededRandom(42)
            for (i in 0 until 500) {
                val body = rigidBody()
                    .id(i)
                    .bodyType(BodyType.DYNAMIC)
                    .position(Vector3(
                        random.nextDouble(-20.0, 20.0),
                        random.nextDouble(5.0, 50.0),
                        random.nextDouble(-20.0, 20.0)
                    ))
                    .mass(random.nextDouble(0.5, 5.0))
                    .build()
                pipeline.addBody(body)
                pipeline.setShape(i, BoxShape(Vector3(0.5, 0.5, 0.5)))
            }
        }
    }

    /**
     * Benchmark 3: 1,000 rigid bodies.
     */
    fun benchmark1000Bodies(): BenchmarkResult {
        val config = PhysicsConfiguration(
            gravity = Vector3(0.0, -9.81, 0.0),
            fixedTimestep = 1.0 / 60.0,
            maxSolverIterations = 10,
            linearDamping = 0.01,
            angularDamping = 0.01,
            allowSleeping = false
        )

        return runBenchmark("1000 Bodies", 1000, 200, config) { pipeline ->
            val random = SeededRandom(42)
            for (i in 0 until 1000) {
                val body = rigidBody()
                    .id(i)
                    .bodyType(BodyType.DYNAMIC)
                    .position(Vector3(
                        random.nextDouble(-30.0, 30.0),
                        random.nextDouble(5.0, 80.0),
                        random.nextDouble(-30.0, 30.0)
                    ))
                    .mass(random.nextDouble(0.5, 5.0))
                    .build()
                pipeline.addBody(body)
                pipeline.setShape(i, BoxShape(Vector3(0.5, 0.5, 0.5)))
            }
        }
    }

    /**
     * Benchmark 4: Dense collision scenario — 100 bodies in a small area.
     */
    fun benchmarkDenseCollision(): BenchmarkResult {
        val config = PhysicsConfiguration(
            gravity = Vector3(0.0, -9.81, 0.0),
            fixedTimestep = 1.0 / 60.0,
            maxSolverIterations = 10,
            linearDamping = 0.01,
            angularDamping = 0.01,
            allowSleeping = false
        )

        return runBenchmark("Dense Collision (100 bodies)", 100, 300, config) { pipeline ->
            val random = SeededRandom(42)
            for (i in 0 until 100) {
                val body = rigidBody()
                    .id(i)
                    .bodyType(BodyType.DYNAMIC)
                    .position(Vector3(
                        random.nextDouble(-2.0, 2.0),
                        random.nextDouble(0.5, 10.0),
                        random.nextDouble(-2.0, 2.0)
                    ))
                    .mass(random.nextDouble(0.5, 2.0))
                    .build()
                pipeline.addBody(body)
                pipeline.setShape(i, BoxShape(Vector3(0.3, 0.3, 0.3)))
            }
        }
    }

    /**
     * Benchmark 5: Large body stack — 50 cubes stacked.
     */
    fun benchmarkLargeStack(): BenchmarkResult {
        val config = PhysicsConfiguration(
            gravity = Vector3(0.0, -9.81, 0.0),
            fixedTimestep = 1.0 / 60.0,
            maxSolverIterations = 10,
            linearDamping = 0.01,
            angularDamping = 0.01,
            allowSleeping = false
        )

        return runBenchmark("Large Stack (50 cubes)", 50, 600, config) { pipeline ->
            val halfExtent = 0.5
            for (i in 0 until 50) {
                val y = halfExtent + i * (halfExtent * 2 + 0.01)
                val body = rigidBody()
                    .id(i)
                    .bodyType(BodyType.DYNAMIC)
                    .position(Vector3(0.0, y.toDouble(), 0.0))
                    .mass(1.0)
                    .build()
                pipeline.addBody(body)
                pipeline.setShape(i, BoxShape(Vector3(halfExtent, halfExtent, halfExtent)))
            }
        }
    }

    /**
     * Benchmark 6: Mixed collision shapes — 50 bodies with Box, Sphere, Capsule, ConvexHull.
     */
    fun benchmarkMixedShapes(): BenchmarkResult {
        val config = PhysicsConfiguration(
            gravity = Vector3(0.0, -9.81, 0.0),
            fixedTimestep = 1.0 / 60.0,
            maxSolverIterations = 10,
            linearDamping = 0.01,
            angularDamping = 0.01,
            allowSleeping = false
        )

        return runBenchmark("Mixed Shapes (50 bodies)", 50, 300, config) { pipeline ->
            val random = SeededRandom(42)
            for (i in 0 until 50) {
                val body = rigidBody()
                    .id(i)
                    .bodyType(BodyType.DYNAMIC)
                    .position(Vector3(
                        random.nextDouble(-5.0, 5.0),
                        random.nextDouble(5.0, 20.0),
                        random.nextDouble(-5.0, 5.0)
                    ))
                    .mass(random.nextDouble(0.5, 3.0))
                    .build()
                pipeline.addBody(body)

                when (i % 4) {
                    0 -> pipeline.setShape(i, BoxShape(Vector3(0.5, 0.5, 0.5)))
                    1 -> pipeline.setShape(i, SphereShape(Vector3.ZERO, 0.5))
                    2 -> pipeline.setShape(i, CapsuleShape(Vector3(0.0, -0.3, 0.0), Vector3(0.0, 0.3, 0.0), 0.3))
                    3 -> pipeline.setShape(i, ConvexHullShape(
                        listOf(
                            Vector3(-0.5, -0.5, -0.5),
                            Vector3(0.5, -0.5, -0.5),
                            Vector3(0.5, -0.5, 0.5),
                            Vector3(-0.5, -0.5, 0.5),
                            Vector3(-0.5, 0.5, -0.5),
                            Vector3(0.5, 0.5, -0.5),
                            Vector3(0.5, 0.5, 0.5),
                            Vector3(-0.5, 0.5, 0.5)
                        )
                    ))
                }
            }
        }
    }

    /**
     * Benchmark 7: High-speed CCD scenario — 10 fast bodies vs static obstacles.
     */
    fun benchmarkCCD(): BenchmarkResult {
        val config = PhysicsConfiguration(
            gravity = Vector3(0.0, -9.81, 0.0),
            fixedTimestep = 1.0 / 60.0,
            maxSolverIterations = 10,
            linearDamping = 0.01,
            angularDamping = 0.01,
            ccdEnabled = true,
            ccdVelocityThreshold = 10.0,
            ccdMaxSubsteps = 20,
            ccdToiTolerance = 1e-6,
            allowSleeping = false
        )

        return runBenchmark("CCD (10 fast bodies)", 10, 300, config) { pipeline ->
            // Static obstacles
            for (i in 0 until 5) {
                val obstacle = rigidBody()
                    .id(100 + i)
                    .bodyType(BodyType.STATIC)
                    .position(Vector3(-10.0 + i * 5.0, 2.0, 0.0))
                    .mass(0.0)
                    .build()
                pipeline.addBody(obstacle)
                pipeline.setShape(100 + i, BoxShape(Vector3(1.0, 1.0, 2.0)))
            }

            // Fast dynamic bodies
            val random = SeededRandom(42)
            for (i in 0 until 10) {
                val body = rigidBody()
                    .id(i)
                    .bodyType(BodyType.DYNAMIC)
                    .position(Vector3(
                        random.nextDouble(-10.0, 10.0),
                        random.nextDouble(20.0, 50.0),
                        0.0
                    ))
                    .mass(1.0)
                    .build()
                // Give them high downward velocity
                body.linearVelocity = Vector3(0.0, -50.0, 0.0)
                pipeline.addBody(body)
                pipeline.setShape(i, BoxShape(Vector3(0.3, 0.3, 0.3)))
            }
        }
    }

    /**
     * Benchmark 8: Long-running simulation — 100 bodies, 1000 steps.
     */
    fun benchmarkLongRunning(): BenchmarkResult {
        val config = PhysicsConfiguration(
            gravity = Vector3(0.0, -9.81, 0.0),
            fixedTimestep = 1.0 / 60.0,
            maxSolverIterations = 10,
            linearDamping = 0.01,
            angularDamping = 0.01,
            allowSleeping = false
        )

        return runBenchmark("Long Running (100 bodies, 1000 steps)", 100, 1000, config) { pipeline ->
            val random = SeededRandom(42)
            for (i in 0 until 100) {
                val body = rigidBody()
                    .id(i)
                    .bodyType(BodyType.DYNAMIC)
                    .position(Vector3(
                        random.nextDouble(-10.0, 10.0),
                        random.nextDouble(5.0, 30.0),
                        random.nextDouble(-10.0, 10.0)
                    ))
                    .mass(random.nextDouble(0.5, 5.0))
                    .build()
                pipeline.addBody(body)
                pipeline.setShape(i, BoxShape(Vector3(0.5, 0.5, 0.5)))
            }
        }
    }

    /**
     * Run all benchmarks and return results.
     */
    fun runAll(): List<BenchmarkResult> {
        return listOf(
            benchmark100Bodies(),
            benchmark500Bodies(),
            benchmark1000Bodies(),
            benchmarkDenseCollision(),
            benchmarkLargeStack(),
            benchmarkMixedShapes(),
            benchmarkCCD(),
            benchmarkLongRunning()
        )
    }
}

/**
 * Main entry point for running benchmarks from command line.
 */
fun main() {
    println("=== Vakuum Physics Engine Benchmark Suite ===")
    println("Hardware: ${System.getProperty("os.name")} ${System.getProperty("os.arch")}")
    println("JVM: ${System.getProperty("java.version")} (${System.getProperty("java.vendor")})")
    println("CPU Cores: ${Runtime.getRuntime().availableProcessors()}")
    println("Max Memory: ${Runtime.getRuntime().maxMemory() / 1024 / 1024} MB")
    println()

    val suite = BenchmarkSuite()
    val results = suite.runAll()

    for (result in results) {
        println(result)
        println()
    }

    // Summary table
    println("=== Summary ===")
    println(String.format("%-35s %6s %10s %10s %10s", "Benchmark", "Bodies", "Avg(ms)", "P95(ms)", "Total(ms)"))
    println("-".repeat(75))
    for (r in results) {
        println(String.format("%-35s %6d %10.4f %10.4f %10.2f", r.name, r.bodyCount, r.avgStepTimeMs, r.p95StepTimeMs, r.totalSimulationTimeMs))
    }
}
