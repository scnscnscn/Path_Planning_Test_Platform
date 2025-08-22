# GitHub Copilot Instructions for Path_Planning_Test_Platform

## Project Overview

This repository is a TurtleBot3-based path planning test platform built on ROS2 Humble. It includes official TurtleBot3 packages (like turtlebot3_description, turtlebot3_navigation2) and custom path planners. The project aims to expand with an "algorithm performance testing framework" to validate both functional correctness and performance of path planning algorithms.

**Maintainer Information:**
- Email: WLQVincent@gmail.com
- GitHub: scnscnscn
- When code or documentation requires maintainer information, use the above details.

## Code Style and Formatting

### C++ Code Standards
All C++ code must strictly follow these clang-format rules:

```yaml
BasedOnStyle: Google
IndentWidth: 2                    # 2 spaces indentation
ColumnLimit: 0                    # No line length limit
AccessModifierOffset: -2          # Access modifiers (public/private) offset by -2
PointerAlignment: Left            # Pointer symbol (*) left-aligned
AlignTrailingComments: true       # Align trailing comments
AllowShortIfStatementsOnASingleLine: false  # No short if statements on single line
AllowShortLoopsOnASingleLine: false         # No short loops on single line
BinPackArguments: true            # Pack function arguments (compact multi-line layout)
BinPackParameters: true           # Pack function parameters (compact multi-line layout)
BreakBeforeBraces: Allman         # Braces ({}) on separate lines (Allman style)
DerivePointerAlignment: false     # Don't auto-derive pointer alignment
NamespaceIndentation: All         # Indent all namespace content
ReflowComments: true              # Auto-reformat comments
SortIncludes: false               # Don't auto-sort include headers
TabWidth: 2                       # Tab width equals 2 spaces
UseTab: Never                     # Never use tabs, only spaces
```

**Code Formatting Commands:**
- Format C++ files: `clang-format -i <file>`
- Check format compliance before committing

### Python Code Standards
Follow PEP 8 standards strictly:
- Use `black` for automatic formatting: `black <file>`
- Use `flake8` for style checking: `flake8 <file>`
- Fix all style violations before committing

## Repository Structure

**Maintain existing repository structure strictly:**
- Do not add/remove core directories without explicit approval
- Do not modify directory purposes arbitrarily
- Follow established file organization patterns

**Key Directories:**
- `custom_planners/` - Custom path planning algorithms
- `docs/` - Project documentation
- `test/` - Unit tests and testing framework
- Official TurtleBot3 packages (maintain as-is)

## Development Guidelines

### Dependency Injection Pattern
- Use dependency injection for appropriate scenarios (planner instances, test framework decoupling)
- Improve code extensibility and testability
- Example: Pass planner instances rather than hard-coding dependencies

### Unit Testing Requirements
- **All new features must include unit tests**
- Prefer `gtest` framework for C++ testing
- Test coverage must include:
  - Core logic functionality
  - Exception/error scenarios
  - Edge cases
- Test files should follow naming: `test_<component>_<function>.cpp`

### Documentation Standards
- Document all public APIs (e.g., planner `compute_path` functions)
- Document complex business logic (e.g., path smoothing algorithms)
- Update `docs/` directory when adding new features
- Include detailed comments for complex algorithms
- API documentation should include:
  - Function purpose
  - Parameter descriptions
  - Return value specifications
  - Usage examples

## Branch and Commit Standards

### Branch Naming Convention
Follow `type/feature-name` format:
- **New features:** `feature/add-rrt-planner`
- **Bug fixes:** `fix/astar-obstacle-crash`
- **Testing:** `test/add-teb-time-test`
- **Documentation:** `docs/update-api-reference`

### Commit Message Format
Use prefixes to indicate operation type:
- `[FEATURE] <description>` - New functionality
- `[FIX] <description>` - Bug fixes
- `[DOC] <description>` - Documentation updates
- `[TEST] <description>` - Testing improvements
- `[REFACTOR] <description>` - Code refactoring
- `[CONFIG] <description>` - Configuration changes

**Examples:**
- `[FEATURE] Add RRT planner with gtest unit tests (path: custom_planners/rrt/)`
- `[DOC] Update docs/planners_api.md with A* planner parameter descriptions`
- `[TEST] Fix path length calculation in test_astar_planner.cpp`

### Pull Request Requirements

**PR Reviewer:** Always assign `scnscnscn` as reviewer

**PR Description must include:**

1. **Functionality/Change Description:**
   - Clear explanation of implemented features or fixed issues
   - Example: "Added Dijkstra planner" or "Fixed TEB planner path oscillation in narrow corridors"

2. **Local Verification Results:**
   - Build status: `colcon build` success/failure
   - Test status: `colcon test` results
   - Any unresolved warnings/errors

3. **Documentation Updates:**
   - Specify if `docs/` directory was modified
   - Example: "Updated docs/test_framework.md with timing test module usage"
   - If no documentation changes: "No documentation updates"

## Dependency Management

### C++ Dependencies
- Add new dependencies in `package.xml` files
- Example: For nav2_core navigation package:
  ```xml
  <depend>nav2_core</depend>
  ```
- Update CMakeLists.txt accordingly
- Document dependency rationale in commit messages

### ROS2 Package Structure
- Follow standard ROS2 package layout
- Include proper package.xml metadata
- Maintain launch file organization
- Keep configuration files in appropriate directories

## Testing Framework

### Unit Test Structure
- Place tests in `test/` directory
- Follow naming convention: `test_<component>.cpp`
- Include both positive and negative test cases
- Test public API functions thoroughly
- Mock external dependencies when appropriate

### Performance Testing
- Implement timing tests for algorithm comparison
- Document performance benchmarks
- Include test data and expected results
- Measure and report algorithm execution times

## Code Review Guidelines

### Before Submitting Code
1. Run clang-format on all C++ files
2. Run black and flake8 on all Python files
3. Execute `colcon build` to ensure compilation
4. Run `colcon test` to verify all tests pass
5. Update documentation if APIs changed
6. Write descriptive commit messages

### Prohibited Actions
- Modifying official TurtleBot3 package source code
- Changing existing functionality logic without explicit approval
- Adding unnecessary dependencies
- Ignoring code formatting standards
- Submitting code without tests for new features

## File Organization

### Code Structure
- Keep related functionality together
- Use clear, descriptive file names
- Organize headers and implementations properly
- Follow ROS2 naming conventions

### Documentation Structure
```
docs/
├── api/                 # API documentation
├── tutorials/           # User guides
├── development/         # Development guides
└── testing/            # Testing documentation
```

## Quality Assurance

### Code Quality Checklist
- [ ] Code follows formatting standards
- [ ] Unit tests written and passing
- [ ] Documentation updated
- [ ] No compiler warnings
- [ ] Dependency injection used appropriately
- [ ] Public APIs documented
- [ ] Error handling implemented
- [ ] Performance considerations addressed

### Pre-commit Validation
1. Format check: `clang-format --dry-run --Werror <files>`
2. Python format: `black --check <files>`
3. Build test: `colcon build`
4. Unit tests: `colcon test`
5. Documentation build (if applicable)

## Additional Notes

- When in doubt about project structure or standards, refer to existing code patterns
- Prioritize code readability and maintainability
- Use meaningful variable and function names
- Include proper error handling and logging
- Consider performance implications of changes
- Follow ROS2 best practices throughout development

Remember: The goal is to maintain a high-quality, well-documented, and thoroughly tested path planning test platform that serves as a reliable foundation for algorithm development and validation.