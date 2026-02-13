## [1.1.1](https://github.com/linglitel/ZeroForm_FOC/compare/v1.1.0...v1.1.1) (2026-02-13)

### ⚠ BREAKING CHANGES

* Remove cogging compensation module and simplify flash storage

- Remove app_cogging.h/c module entirely
- Simplify flash storage to only save electrical angle offset
- Simplify CAN protocol, remove unused commands (CMD_CALIBRATE 0x8)
- Add calibration via CONFIG command (0x02) with auto Flash save
- Update CAN protocol documentation to V1.1
- Remove motor config from CAN (pairs, direction, vbus)
- Simplify UART commands, remove SAVE/COGGING commands

### Refactoring

* simplify flash storage and CAN protocol ([3099247](https://github.com/linglitel/ZeroForm_FOC/commit/309924765bf9c68bb450c2703390aeab4e0e7e14))

## [1.1.0](https://github.com/linglitel/ZeroForm_FOC/compare/v1.0.1...v1.1.0) (2026-02-13)

### Features

* **flash:** add flash storage, cogging compensation and feedforward control ([c2fb66e](https://github.com/linglitel/ZeroForm_FOC/commit/c2fb66eea259c75cb1fd5af9f5d1325ce4d02a66))

## [1.0.1](https://github.com/linglitel/ZeroForm_FOC/compare/v1.0.0...v1.0.1) (2026-02-12)

### Bug Fixes

* **ci:** add missing conventional-changelog-conventionalcommits dependency ([160ed80](https://github.com/linglitel/ZeroForm_FOC/commit/160ed8079de097ef63a1494c0c0d23840c2a455d))
