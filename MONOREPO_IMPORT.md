# Monorepo import

The ROS packages under `src/` used to be git submodules. They were flattened into
this repository as a snapshot: their per-package history remains only in the
upstream remotes listed below, at the commit each package was imported from.

`src/rclgd` is deliberately still a submodule — it is an independent, externally
published project (and carries its own `godot-cpp` submodule).

| Package | Imported commit | Upstream |
|---|---|---|
| `src/mirena_bringup` | `0e57ab102d56` | https://git.eracing.es/Driverless/mirena_bringup |
| `src/mirena_common` | `421a37f484f4` | https://git.eracing.es/Driverless/mirena_common |
| `src/mirena_consensus` | `a5b28f8d8d38` | https://git.eracing.es/Driverless/mirena_consensus |
| `src/mirena_control` | `dfab408deaba` | https://git.eracing.es/Driverless/mirena_control |
| `src/mirena_hardware` | `68c6c018d8ba` | https://git.eracing.es/Driverless/mirena_hardware |
| `src/mirena_monitor` | `889c679d6634` | https://git.eracing.es/Driverless/mirena_monitor |
| `src/mirena_perception` | `0887b9821374` | https://git.eracing.es/Driverless/mirena_perception |
| `src/mirena_planning` | `5472baece078` | https://git.eracing.es/Driverless/mirena_planning |
| `src/mirena_remote` | `2a3a474c9de1` | https://git.eracing.es/Driverless/mirena_remote.git |
| `src/mirena_rviz2_plugins` | `66748343f0bb` | https://git.eracing.es/Driverless/mirena_rviz2_plugins |
| `src/mirena_sim` | `67f456859ef2` | https://git.eracing.es/Driverless/mirena_sim |
| `src/mirena_slam` | `9609906b681d` | https://git.eracing.es/Driverless/mirena_slam |
| `src/mirena_supervisor` | `3ec7c52a4c6e` | https://git.eracing.es/Driverless/mirena_supervisor |
| `src/mirena_utilities` | `73ec25ad04e9` | https://git.eracing.es/Driverless/mirena_utilities |

## Not imported

`src/mirena_monitor/core.2976` — a 1.0 GB core dump that was tracked upstream and
already deleted in the working tree. It is reachable in the `mirena_monitor`
upstream at the commit above; it was left out rather than welded into this history.

`.gitmodules` also carried a stale `src/mirena_tests` entry for a directory that
did not exist. It was dropped.
