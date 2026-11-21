package io.vakuum.runner

import io.vakuum.core.PhysicsConfiguration
import io.vakuum.dynamics.RigidBody
import io.vakuum.math.Vector3
import java.io.File

/**
 * Exports simulation data to JSON for Blender import.
 */
class SimulationExporter(private val config: PhysicsConfiguration) {

    private data class FrameData(
        val frame: Int,
        val time: Double,
        val bodies: List<BodyData>
    )

    private data class BodyData(
        val id: Int,
        val position: Vector3,
        val orientation: io.vakuum.math.Quaternion,
        val linVel: Vector3,
        val angVel: Vector3
    )

    private val frames = mutableListOf<FrameData>()

    /**
     * Record the current state of all bodies.
     */
    fun record(frame: Int, time: Double, bodies: List<RigidBody>) {
        val bodyData = bodies.map { body ->
            BodyData(
                id = body.id,
                position = body.transform.position,
                orientation = body.transform.rotation,
                linVel = body.linearVelocity,
                angVel = body.angularVelocity
            )
        }
        frames.add(FrameData(frame, time, bodyData))
    }

    /**
     * Export the recorded frames to a JSON file.
     */
    fun export(filePath: String) {
        val file = File(filePath)
        file.parentFile?.mkdirs()

        file.writeText(buildJson())
    }

    private fun buildJson(): String {
        val sb = StringBuilder()
        sb.appendLine("{")
        sb.appendLine("  \"metadata\": {")
        sb.appendLine("    \"timestep\": ${config.fixedTimestep},")
        sb.appendLine("    \"gravity\": [${config.gravity.x}, ${config.gravity.y}, ${config.gravity.z}],")
        sb.appendLine("    \"frameCount\": ${frames.size}")
        sb.appendLine("  },")
        sb.appendLine("  \"frames\": [")

        for (i in frames.indices) {
            val frame = frames[i]
            sb.appendLine("    {")
            sb.appendLine("      \"frame\": ${frame.frame},")
            sb.appendLine("      \"time\": ${frame.time},")
            sb.appendLine("      \"bodies\": [")

            for (j in frame.bodies.indices) {
                val body = frame.bodies[j]
                val q = body.orientation
                sb.appendLine("        {")
                sb.appendLine("          \"id\": ${body.id},")
                sb.appendLine("          \"position\": [${body.position.x}, ${body.position.y}, ${body.position.z}],")
                sb.appendLine("          \"orientation\": [${q.w}, ${q.x}, ${q.y}, ${q.z}],")
                sb.appendLine("          \"linVel\": [${body.linVel.x}, ${body.linVel.y}, ${body.linVel.z}],")
                sb.appendLine("          \"angVel\": [${body.angVel.x}, ${body.angVel.y}, ${body.angVel.z}]")
                sb.appendLine("        }${if (j < frame.bodies.size - 1) "," else ""}")
            }

            sb.appendLine("      ]")
            sb.appendLine("    }${if (i < frames.size - 1) "," else ""}")
        }

        sb.appendLine("  ]")
        sb.appendLine("}")
        return sb.toString()
    }
}
