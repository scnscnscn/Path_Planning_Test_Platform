# GitHub Workflows

This directory contains GitHub Actions workflows for the Path Planning Test Platform.

## Available Workflows

### Continuous Integration (CI)
- **File:** `ci.yml`
- **Purpose:** Automated testing and validation for pull requests
- **Triggers:** Push to main branch, pull requests
- **Actions:**
  - Code formatting validation (clang-format, black)
  - Build verification (colcon build)
  - Unit test execution (colcon test)
  - Documentation build checks

### Code Quality Checks
- **File:** `code-quality.yml`
- **Purpose:** Automated code quality and style checks
- **Triggers:** Pull requests
- **Actions:**
  - C++ style checking with clang-format
  - Python style checking with flake8
  - Documentation link validation
  - License header verification

### Performance Benchmarking
- **File:** `benchmark.yml`
- **Purpose:** Automated performance testing and comparison
- **Triggers:** Release tags, manual dispatch
- **Actions:**
  - Run algorithm performance benchmarks
  - Generate performance reports
  - Compare against baseline metrics
  - Upload benchmark artifacts

## Workflow Configuration

### Required Secrets
- None currently required for public repository

### Environment Variables
- `ROS_DISTRO`: ROS2 distribution (humble)
- `UBUNTU_VERSION`: Ubuntu version (22.04)

### Dependencies
- ROS2 Humble packages
- Build tools (colcon, cmake)
- Testing frameworks (gtest, benchmark)
- Code quality tools (clang-format, flake8, black)

## Workflow Status

Current status of automated workflows:

- ✅ Basic structure created
- ⏳ CI workflow implementation pending
- ⏳ Code quality checks pending
- ⏳ Performance benchmarking pending

## Adding New Workflows

When adding new workflows:

1. Follow GitHub Actions best practices
2. Use appropriate triggers and conditions
3. Include proper error handling
4. Add status badges to README
5. Document any new requirements
6. Test workflows on feature branches

## Maintainer Notes

**Workflow Maintenance:**
- Review and update dependencies regularly
- Monitor workflow execution times
- Optimize for cost and performance
- Keep security considerations in mind

**Contact:** WLQVincent@gmail.com for workflow-related questions.