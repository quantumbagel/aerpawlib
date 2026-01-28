# aerpawlib roadmap

There are a number of features that are not currently implemented in aerpawlib but are planned for future releases. This roadmap outlines some of the key features and improvements we aim to add:


- v2 initial release
  - Safety monitor is VERY buggy
  - Connection lifecycle handling is buggy and will sometimes report heartbeat lost incorrectly
  - Add more integration tests for v2 API
  - Fix connection handling on quit, it seems to be an infinite loop because of the usage of signal
  - Improve documentation with more examples and tutorials
  - Better OEO integration features
- v1 API improvements
  - OEO integration for v1 API