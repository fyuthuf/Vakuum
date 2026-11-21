# Vakuum.xyz — Website Integration

## Overview

The `vakuum-website` module provides an integration adapter for embedding the Vakuum physics engine in the Vakuum.xyz website. It isolates simulation sessions, manages rigid bodies, and provides a clean API for real-time interaction.

## Architecture

```
Browser (WebSocket Client)
    ↕ JSON messages
WebSocket Server (future)
    ↕ Kotlin API
vakuum-website (SimulationSession)
    ↕ Physics calculations
vakuum-runner (SimulationPipeline)
    ↕ Collision, solving, integration
vakuum-core / vakuum-dynamics / vakuum-collision / vakuum-solver
```

The actual Vakuum engine remains authoritative. The website does not implement its own physics simulation.

## Module Structure

```
vakuum-website/
├── pom.xml
└── src/main/kotlin/io/vakuum/website/
    ├── BodyState.kt          # Body state snapshot + diagnostics
    └── SimulationSession.kt  # Isolated session management
```

## API Reference

### Creating a Session

```kotlin
val session = SimulationSessionFactory.createSession()
```

Each session is isolated with its own `SimulationPipeline`, bodies, and configuration.

### Setting Up Boundaries

```kotlin
session.setupBoundaries()  // Creates 4 static boundary walls
```

Creates a bounded physics environment with ground, left, right, and back walls. Used for the ASCII Playground.

### Spawning Bodies

```kotlin
// Box
val boxId = session.spawnBox(
    position = Vector3(0.0, 5.0, 0.0),
    halfExtents = Vector3(0.5, 0.5, 0.5),
    mass = 1.0,
    restitution = 0.3,
    friction = 0.5
)

// Sphere
val sphereId = session.spawnSphere(
    position = Vector3(0.0, 10.0, 0.0),
    radius = 0.5,
    mass = 1.0
)
```

### Grabbing Bodies

```kotlin
// Grab a body and move it toward a target
session.grabBody(bodyId, Vector3(2.0, 3.0, 0.0))

// Update the grab target
session.updateGrabTarget(bodyId, Vector3(3.0, 4.0, 0.0))

// Release — body retains its velocity
session.releaseBody(bodyId)
```

The grab constraint smoothly moves the body toward the target position each simulation step. On release, the body retains the velocity accumulated during the grab.

### Applying Forces and Impulses

```kotlin
session.applyForce(bodyId, Vector3(0.0, -10.0, 0.0))
session.applyImpulse(bodyId, Vector3(0.0, 5.0, 0.0))
```

### Simulation Control

```kotlin
session.step()     // Execute one simulation step
session.pause()    // Pause the simulation
session.resume()   // Resume
session.reset()    // Reset state
session.destroy()  // Clean up
```

### Retrieving State

```kotlin
// All bodies
val states: List<BodyState> = session.getBodyStates()

// Specific body
val state: BodyState? = session.getBodyState(bodyId)

// Diagnostics
val diag: SimulationDiagnostics = session.getDiagnostics()
```

### BodyState

```kotlin
data class BodyState(
    val id: Int,
    val position: Vector3,
    val rotation: Quaternion,
    val linearVelocity: Vector3,
    val angularVelocity: Vector3,
    val mass: Double,
    val isAwake: Boolean
)
```

### SimulationDiagnostics

```kotlin
data class SimulationDiagnostics(
    val bodyCount: Int,
    val stepCount: Long,
    val contactCount: Int,
    val ccdSteps: Int,
    val toiQueries: Int,
    val hasNaN: Boolean
)
```

## WebSocket Protocol (Planned)

The website communication layer uses WebSocket for real-time updates. The protocol is not implemented in this milestone but the API is designed to support it.

### Request Format (JSON)

```json
{
    "type": "spawn_box",
    "sessionId": "session-123",
    "position": {"x": 0.0, "y": 5.0, "z": 0.0},
    "mass": 1.0
}
```

### Response Format (JSON)

```json
{
    "type": "body_state",
    "sessionId": "session-123",
    "bodyId": 0,
    "position": {"x": 0.0, "y": 3.27, "z": 0.0},
    "velocity": {"x": 0.0, "y": -5.88, "z": 0.0}
}
```

## Security Considerations

- **Isolated sessions**: Each session has its own pipeline; sessions cannot interfere with each other.
- **Resource limits**: Maximum bodies per session should be enforced at the server level.
- **Message validation**: All incoming messages should be validated before processing.
- **Rate limiting**: WebSocket messages should be rate-limited to prevent abuse.
- **No arbitrary operations**: The API exposes only physics operations, not server internals.

## ASCII Playground

The ASCII Playground uses the website adapter to provide an interactive physics environment:

1. **Bounded environment**: 4 static boundary walls created via `setupBoundaries()`.
2. **Dynamic cubes**: Spawned via `spawnBox()`.
3. **Mouse-driven grab**: `grabBody()` and `updateGrabTarget()` move bodies toward the mouse cursor.
4. **Natural release**: `releaseBody()` lets the body retain its velocity.
5. **Gravity and collision**: Handled by the Vakuum engine.
6. **Reset**: `reset()` clears the simulation.

The browser handles input and rendering. The physics engine handles all calculations. Bodies are never teleported to mouse coordinates — they follow the grab target smoothly.

## Known Limitations

- `removeBody()` does not actually remove the body from the pipeline (no `removeBody` method in `SimulationPipeline`). The body is deactivated instead.
- Full `reset()` requires recreating the pipeline. Currently only clears grab targets and step count.
- WebSocket protocol is not implemented; the adapter provides the Kotlin API only.
- Single-threaded; concurrent session access requires external synchronization.
