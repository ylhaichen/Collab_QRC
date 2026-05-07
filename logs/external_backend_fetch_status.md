# External Backend Fetch Status

- Swarm-LIO2: source fetched at `external/Swarm-LIO2` (`a5f751a`); buildable source detected; runtime artifact not installed.
- Dynamic-LIO: source fetched at `external/dynamic_lio` (`4e91af4`); buildable source detected; runtime artifact not installed.
- ERASOR: source fetched at `external/ERASOR` (`ad7e4dc`); buildable source detected; runtime artifact not installed.

Initial sandbox clone failed with `Could not resolve host: github.com`. Escalated fetch partially hit a transient ERASOR error: `Failed to connect to github.com port 443 after 1024 ms: Connection refused`. A single ERASOR retry succeeded.

Current blocker: external sources are available on host, but native backend runtime artifacts are not installed. Fast-LIO remains the production backend.
