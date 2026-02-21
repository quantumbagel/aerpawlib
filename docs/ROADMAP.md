# aerpawlib Roadmap

Planned features and improvements for future releases.

## v1 API (Stable, no breaking changes)

- [ ] **Documentation**
  - [x] Centralize documentation in `docs/`
  - [x] User guide for workflows and features
  - [x] Development guide for contributors
  - [ ] Additional examples and tutorials
- [ ] **Testing**
  - [x] Fix integrated SITL testing (currently broken)
  - [ ] Get all tests passing (currently some skipped/failing)
  - [ ] Installing aerpawlib[dev] installs ardupilot SITL into the code (note that this is the special aerpaw sitl)
  - [ ] ZMQ tests
  - [ ] Get SITL from C-VM docker and use it for testing instead of ardupilot SITL
- [ ] **Features**
  - [ ] ZMQ doesn't appear to be working correctly

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
