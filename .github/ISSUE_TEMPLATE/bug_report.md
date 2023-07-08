---
name: Bug report
about: Create a report to help us improve
title: ''
labels: bug
assignees: ''

---

<!--
For general questions, please ask on https://github.com/stella-cv/stella_vslam/discussions.
Please complete the following information.
-->

## Describe the bug
<!-- A clear and concise description of what the bug is. -->

## To Reproduce

<!-- Edit the following templates -->

When running ... I get "<error>"

shell session:

```shell-session
foo@~/lib/stella_vslam/build$ command
some output ...
"<error>"
```

build options:

```
foo@~/lib/stella_vslam/build: cat CMakeCache.txt | grep -e USE_ -e BUILD_
BUILD_SHARED_LIBS:BOOL=ON
BUILD_TESTS:BOOL=OFF
BUILD_WITH_MARCH_NATIVE:BOOL=OFF
CMAKE_BUILD_TYPE:STRING=RelWithDebInfo
USE_ARUCO:BOOL=ON
USE_CCACHE:BOOL=ON
USE_GOOGLE_PERFTOOLS:BOOL=OFF
USE_GTSAM:BOOL=ON
USE_OPENMP:BOOL=OFF
USE_SANITIZER:BOOL=OFF
USE_SSE_FP_MATH:BOOL=OFF
USE_SSE_ORB:BOOL=OFF
```

config file:

```yaml
Camera:
  model: ...
```

## Expected behavior
<!-- A clear and concise description of what you expected to happen. -->

## Screenshots or videos
<!-- If applicable, add screenshots to help explain your problem. -->

## Environment

- Hardware: [e.g. PC, Raspberry Pi 4, Jetson Nano]
- CPU: [e.g. Intel Core i7-11700]
- OS: [e.g. Ubuntu 20.04]
- Commit id: [e.g. stella_vslam=577cb04d9245aa68b0905212293a802292d01787, stella_vslam_ros=fdbd287d80a3a5cc4ac1a143dc937adb4697b120]
- Install procedure: [docker or native]
- dataset: [e.g. EuRoC MH_01]

## Additional context
<!-- Add any other context about the problem here. -->
