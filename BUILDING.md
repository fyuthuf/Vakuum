# Building Vakuum

## Requirements

- **JDK 21** (OpenJDK, Temurin, or Zulu)
- **Maven 3.8+** (or use the Maven Wrapper)

## Building

### With Maven Wrapper (recommended)

```bash
# Linux/macOS
./mvnw clean verify

# Windows
mvnw.cmd clean verify
```

### With system Maven

```bash
mvn clean verify
```

## Running Tests

```bash
mvn test
```

## Running Simulations

```bash
# Build the runner JAR
mvn package

# Run Experiment A (single falling cube)
java -cp vakuum-runner/target/vakuum-runner-0.1.0-SNAPSHOT.jar:vakuum-core/target/vakuum-core-0.1.0-SNAPSHOT.jar:vakuum-math/target/vakuum-math-0.1.0-SNAPSHOT.jar:vakuum-dynamics/target/vakuum-dynamics-0.1.0-SNAPSHOT.jar:vakuum-collision/target/vakuum-collision-0.1.0-SNAPSHOT.jar:vakuum-solver/target/vakuum-solver-0.1.0-SNAPSHOT.jar io.vakuum.runner.SimulationRunnerKt A
```

## Module Structure

| Module | Description |
|--------|-------------|
| vakuum-math | Mathematical foundation (vectors, matrices, quaternions) |
| vakuum-core | Physics world lifecycle and configuration |
| vakuum-dynamics | Rigid body dynamics |
| vakuum-collision | Collision detection (broadphase + narrowphase) |
| vakuum-solver | Impulse-based constraint solver |
| vakuum-runner | Simulation runner and JSON export |
| vakuum-tests | Integration and regression tests |
