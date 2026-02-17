# aerpawlib Roadmap

Planned features and improvements for future releases.

## v1 API (Stable, no breaking changes)

- [ ] **Documentation**
  - [x] Centralize documentation in `docs/`
  - [x] User guide for workflows and features
  - [x] Development guide for contributors
  - [ ] Additional examples and tutorials
- [ ] **Testing**
  - [ ] More integration tests for v1 API
  - [ ] Fix testing pipeline (CI, SITL setup)

## v2 API (Development)

- [ ] **Critical fixes**
  - [ ] Safety monitor is very buggy
  - [ ] Connection lifecycle handling (heartbeat lost reported incorrectly)
  - [ ] Fix connection handling on quit (infinite loop with signal usage)
- [ ] **Testing**
  - [ ] Add more integration tests for v2 API
- [ ] **Documentation**
  - [ ] More examples and tutorials for v2
- [ ] **Platform**
  - [ ] Better OEO integration features

## Future Considerations

- v1 → v2 migration tooling or guide
