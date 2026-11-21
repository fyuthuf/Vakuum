package io.vakuum.runner

import io.vakuum.collision.BoxShape
import io.vakuum.core.PhysicsConfiguration
import io.vakuum.dynamics.rigidBody
import io.vakuum.math.Vector3

/**
 * Main simulation runner for experiments.
 *
 * Usage: java -cp vakuum-runner.jar io.vakuum.runner.SimulationRunner <experiment>
 *
 * Experiments:
 *   A - Single falling cube
 *   B - Multiple falling cubes
 *   C - Cube stacking
 *   D - Stress test (100 cubes)
 *   E - Reproducibility test
 */
fun main(args: Array<String>) {
    val experiment = if (args.isNotEmpty()) args[0].uppercase() else "A"
    val outputDir = if (args.size > 1) args[1] else "output"

    val config = PhysicsConfiguration(
        gravity = Vector3(0.0, -9.81, 0.0),
        fixedTimestep = 1.0 / 60.0,
        maxSolverIterations = 10,
        linearDamping = 0.0,
        angularDamping = 0.0
    )

    when (experiment) {
        "A" -> runExperimentA(config, outputDir)
        "B" -> runExperimentB(config, outputDir)
        "C" -> runExperimentC(config, outputDir)
        "D" -> runExperimentD(config, outputDir)
        "E" -> runExperimentE(config, outputDir)
        else -> println("Unknown experiment: $experiment. Use A, B, C, D, or E.")
    }
}

private fun runExperimentA(config: PhysicsConfiguration, outputDir: String) {
    println("Experiment A: Single Falling Cube")
    val pipeline = SimulationPipeline(config)
    val exporter = SimulationExporter(config)

    // Ground plane (static, id=0)
    val ground = rigidBody()
        .id(0)
        .bodyType(io.vakuum.dynamics.BodyType.STATIC)
        .position(0.0, 0.0, 0.0)
        .mass(0.0)
        .build()
    pipeline.addBody(ground)
    pipeline.setShape(0, BoxShape(Vector3(10.0, 0.1, 10.0)))

    // Falling cube (dynamic, id=1)
    val cube = rigidBody()
        .id(1)
        .position(0.0, 10.0, 0.0)
        .mass(1.0)
        .halfExtents(0.5, 0.5, 0.5)
        .restitution(0.3)
        .friction(0.5)
        .build()
    pipeline.addBody(cube)
    pipeline.setShape(1, BoxShape(Vector3(0.5, 0.5, 0.5)))

    val steps = 600
    for (i in 0 until steps) {
        pipeline.step()
        exporter.record(i, i * config.fixedTimestep, pipeline.getBodies())

        if (pipeline.hasNaN()) {
            println("NaN detected at step $i!")
            break
        }
    }

    val filePath = "$outputDir/experiment_a_single_cube.json"
    exporter.export(filePath)
    println("Exported to $filePath")

    val finalCube = pipeline.getBodies().find { it.id == 1 }
    println("Final position: ${finalCube?.transform?.position}")
    println("Final velocity: ${finalCube?.linearVelocity}")
}

private fun runExperimentB(config: PhysicsConfiguration, outputDir: String) {
    println("Experiment B: Multiple Falling Cubes")
    val pipeline = SimulationPipeline(config)
    val exporter = SimulationExporter(config)

    // Ground
    val ground = rigidBody()
        .id(0)
        .bodyType(io.vakuum.dynamics.BodyType.STATIC)
        .position(0.0, 0.0, 0.0)
        .mass(0.0)
        .build()
    pipeline.addBody(ground)
    pipeline.setShape(0, BoxShape(Vector3(10.0, 0.1, 10.0)))

    // 10 cubes at different positions
    for (i in 0 until 10) {
        val x = (i % 5 - 2) * 1.5
        val z = (i / 5 - 1) * 1.5
        val y = 5.0 + i * 2.0
        val mass = 0.5 + i * 0.1

        val cube = rigidBody()
            .id(i + 1)
            .position(x, y, z)
            .mass(mass)
            .halfExtents(0.4, 0.4, 0.4)
            .restitution(0.2)
            .friction(0.5)
            .build()
        pipeline.addBody(cube)
        pipeline.setShape(i + 1, BoxShape(Vector3(0.4, 0.4, 0.4)))
    }

    val steps = 600
    for (i in 0 until steps) {
        pipeline.step()
        exporter.record(i, i * config.fixedTimestep, pipeline.getBodies())

        if (pipeline.hasNaN()) {
            println("NaN detected at step $i!")
            break
        }
    }

    val filePath = "$outputDir/experiment_b_multiple_cubes.json"
    exporter.export(filePath)
    println("Exported to $filePath")
}

private fun runExperimentC(config: PhysicsConfiguration, outputDir: String) {
    println("Experiment C: Cube Stacking")
    val pipeline = SimulationPipeline(config)
    val exporter = SimulationExporter(config)

    // Ground
    val ground = rigidBody()
        .id(0)
        .bodyType(io.vakuum.dynamics.BodyType.STATIC)
        .position(0.0, 0.0, 0.0)
        .mass(0.0)
        .build()
    pipeline.addBody(ground)
    pipeline.setShape(0, BoxShape(Vector3(10.0, 0.1, 10.0)))

    // 4x2 stack of cubes
    var id = 1
    for (row in 0 until 2) {
        for (col in 0 until 4) {
            val x = (col - 1.5) * 1.05
            val y = 0.5 + row * 1.05
            val z = 0.0

            val cube = rigidBody()
                .id(id)
                .position(x, y, z)
                .mass(1.0)
                .halfExtents(0.5, 0.5, 0.5)
                .restitution(0.0)
                .friction(0.8)
                .build()
            pipeline.addBody(cube)
            pipeline.setShape(id, BoxShape(Vector3(0.5, 0.5, 0.5)))
            id++
        }
    }

    val steps = 1200
    for (i in 0 until steps) {
        pipeline.step()
        exporter.record(i, i * config.fixedTimestep, pipeline.getBodies())

        if (pipeline.hasNaN()) {
            println("NaN detected at step $i!")
            break
        }
    }

    val filePath = "$outputDir/experiment_c_stacking.json"
    exporter.export(filePath)
    println("Exported to $filePath")
}

private fun runExperimentD(config: PhysicsConfiguration, outputDir: String) {
    println("Experiment D: Stress Test (100 Cubes)")
    val pipeline = SimulationPipeline(config)
    val exporter = SimulationExporter(config)

    // Ground
    val ground = rigidBody()
        .id(0)
        .bodyType(io.vakuum.dynamics.BodyType.STATIC)
        .position(0.0, 0.0, 0.0)
        .mass(0.0)
        .build()
    pipeline.addBody(ground)
    pipeline.setShape(0, BoxShape(Vector3(50.0, 0.1, 50.0)))

    // 100 cubes
    for (i in 0 until 100) {
        val x = (i % 10 - 5) * 1.2
        val z = (i / 10 - 5) * 1.2
        val y = 5.0 + (i % 20) * 2.0

        val cube = rigidBody()
            .id(i + 1)
            .position(x, y, z)
            .mass(0.5 + (i % 5) * 0.2)
            .halfExtents(0.4, 0.4, 0.4)
            .restitution(0.1)
            .friction(0.5)
            .build()
        pipeline.addBody(cube)
        pipeline.setShape(i + 1, BoxShape(Vector3(0.4, 0.4, 0.4)))
    }

    val steps = 600
    val startTime = System.nanoTime()

    for (i in 0 until steps) {
        pipeline.step()
        exporter.record(i, i * config.fixedTimestep, pipeline.getBodies())

        if (pipeline.hasNaN()) {
            println("NaN detected at step $i!")
            break
        }
    }

    val elapsed = (System.nanoTime() - startTime) / 1_000_000.0
    val filePath = "$outputDir/experiment_d_stress_test.json"
    exporter.export(filePath)
    println("Exported to $filePath")
    println("Time: ${elapsed}ms for $steps steps (${elapsed / steps}ms per step)")
}

private fun runExperimentE(config: PhysicsConfiguration, outputDir: String) {
    println("Experiment E: Reproducibility Test")

    // Run 1
    val pipeline1 = SimulationPipeline(config)
    setupExperimentE(pipeline1)
    val results1 = mutableListOf<String>()
    for (i in 0 until 300) {
        pipeline1.step()
        val bodies = pipeline1.getBodies()
        val cube = bodies.find { it.id == 1 }
        results1.add("${cube?.transform?.position?.x},${cube?.transform?.position?.y},${cube?.transform?.position?.z}")
    }

    // Run 2
    val pipeline2 = SimulationPipeline(config)
    setupExperimentE(pipeline2)
    val results2 = mutableListOf<String>()
    for (i in 0 until 300) {
        pipeline2.step()
        val bodies = pipeline2.getBodies()
        val cube = bodies.find { it.id == 1 }
        results2.add("${cube?.transform?.position?.x},${cube?.transform?.position?.y},${cube?.transform?.position?.z}")
    }

    val identical = results1 == results2
    println("Results identical: $identical")

    if (!identical) {
        for (i in results1.indices) {
            if (results1[i] != results2[i]) {
                println("First difference at frame $i: ${results1[i]} vs ${results2[i]}")
                break
            }
        }
    }
}

private fun setupExperimentE(pipeline: SimulationPipeline) {
    val ground = rigidBody()
        .id(0)
        .bodyType(io.vakuum.dynamics.BodyType.STATIC)
        .position(0.0, 0.0, 0.0)
        .mass(0.0)
        .build()
    pipeline.addBody(ground)
    pipeline.setShape(0, BoxShape(Vector3(10.0, 0.1, 10.0)))

    val cube = rigidBody()
        .id(1)
        .position(0.0, 10.0, 0.0)
        .mass(1.0)
        .halfExtents(0.5, 0.5, 0.5)
        .restitution(0.3)
        .friction(0.5)
        .build()
    pipeline.addBody(cube)
    pipeline.setShape(1, BoxShape(Vector3(0.5, 0.5, 0.5)))
}
