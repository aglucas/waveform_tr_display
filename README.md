# qualia_tr_display

ESP-IDF application for the `esp32p4` platform with a Waveshare LCD touchscreen interface.

## Project structure

- `CMakeLists.txt` — ESP-IDF project entry point
- `main/` — application source code and components
- `components/` — local component code
- `managed_components/` — generated ESP-IDF managed components (ignored by default)
- `dependencies.lock` — pinned managed component dependencies
- `sdkconfig.defaults` — default project configuration

## Build

1. Install a compatible ESP-IDF version (5.x or later).
2. Open a terminal in the project root.
3. Set the target and build:

```bash
idf.py set-target esp32p4
idf.py build
```

## Flash

```bash
idf.py -p <PORT> flash monitor
```

Replace `<PORT>` with your serial port (for example `COM3` on Windows or `/dev/ttyUSB0` on Linux).

## Notes

- `sdkconfig` is ignored; use `sdkconfig.defaults` or run `idf.py menuconfig` to create a local configuration.
- `managed_components/` is ignored by default and can be reconstructed from `dependencies.lock` using ESP-IDF managed component commands.
- Local editor settings such as `.vscode/settings.json` are excluded from version control.
