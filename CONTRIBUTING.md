# Contributing to Vakuum

## Code Style

- Use idiomatic Kotlin with consistent formatting.
- Follow the existing code style in each module.
- Use KDoc for public APIs.
- Keep methods focused and well-named.

## Testing

- All new features must have tests.
- Use JUnit 5 with descriptive test names.
- Test edge cases, not just happy paths.
- Use numerical tolerances for floating-point comparisons.
- No tests that only assert non-null.

## Pull Requests

- Describe the change clearly.
- Reference any related issues.
- Ensure `mvn clean verify` passes.
- Do not introduce external physics engine dependencies.

## Architecture Guidelines

- Keep module dependencies acyclic.
- vakuum-math must remain independent.
- No circular dependencies between modules.
- Public APIs should be stable and well-documented.

## Reporting Bugs

- Include the exact steps to reproduce.
- Include the expected and actual behavior.
- Include the Vakuum version and Java version.
