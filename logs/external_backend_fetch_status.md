# External Backend Fetch Status

- Swarm-LIO2: source fetched at `external/Swarm-LIO2` (`a5f751a`); ROS1/catkin source detected; not buildable on this host because `catkin_make`/`rospack` are unavailable.
- Dynamic-LIO: source fetched at `external/dynamic_lio` (`4e91af4`); ROS1/catkin source detected; not buildable on this host because `catkin_make`/`rospack` are unavailable.
- ERASOR: source fetched at `external/ERASOR` (`ad7e4dc`); ROS1/catkin source detected; not buildable on this host because `catkin_make`/`rospack` are unavailable.

Initial sandbox clone failed with `Could not resolve host: github.com`. Escalated fetch partially hit a transient ERASOR error: `Failed to connect to github.com port 443 after 1024 ms: Connection refused`. A single ERASOR retry succeeded.

Current blocker: external sources are available on host, but native upstream backends require ROS1/catkin tooling that is not exposed here. Fast-LIO remains the production backend.
