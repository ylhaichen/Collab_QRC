# Point-LIO Docker Backend

This image builds the upstream Point-LIO ROS 1 backend from:

```text
https://github.com/hku-mars/Point-LIO
```

The image is a backend runtime/build artifact only. Runtime alignment must flow through the ROS 2 adapter and the DiSCo-style robust loop-closure gate; it must not use ground truth or a hardcoded robot-to-robot transform.

Build/test entrypoint:

```bash
bash scripts/manual/run_point_lio_docker_build_and_test.sh
```
