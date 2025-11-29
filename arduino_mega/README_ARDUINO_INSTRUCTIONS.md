Arduino IntelliSense & build helper

This project contains Arduino sketches for an Arduino Mega and related libraries.
If VS Code shows errors like "cannot open source file \"Arduino.h\"" you need to point the C/C++ extension at your Arduino installation or install the Arduino CLI.

Quick steps (Linux):

1) Install Arduino CLI (recommended):

   curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
   # or use your package manager

2) Initialise Arduino CLI and install core for AVR (if you use AVR Mega):

   ./arduino-cli core update-index
   ./arduino-cli core install arduino:avr

3) Locate Arduino core include paths (example):

   # Arduino CLI installs in ~/.arduino15 by default
   ls -d ~/.arduino15/packages/arduino/hardware/avr/*
   # Common system package locations
   /usr/share/arduino/hardware
   /usr/local/share/arduino

4) If Arduino IDE is unpacked in Downloads (Kali scenario), set ARDUINO_HOME to that folder OR use the CLI packages.

   # Example for IDE extracted to ~/Downloads/arduino-1.8.19
   echo 'export ARDUINO_HOME="$HOME/Downloads/arduino-1.8.19"' >> ~/.zshrc
   # Add AVR toolchain to PATH so IntelliSense finds compiler (optional):
   echo 'export PATH="$HOME/Downloads/arduino-1.8.19/hardware/tools/avr/bin:$PATH"' >> ~/.zshrc
   source ~/.zshrc

4b) Set ARDUINO_HOME (or ARDUINO_PATH) for Arduino CLI packages (example zsh):

   echo 'export ARDUINO_HOME="$HOME/.arduino15"' >> ~/.zshrc
   source ~/.zshrc

5) If you can't install Arduino CLI, update `.vscode/c_cpp_properties.json` includePath to the path where Arduino is installed. Add the `cores/arduino` folder and any `variants/<board>` include folders, plus `/usr/share/arduino/libraries` and `arduino_mega/libraries`.

6) Reload VS Code window after updating settings (Developer: Reload Window) and wait for IntelliSense to re-index.

Troubleshooting (Kali / Downloads install):
- Ensure `.vscode/c_cpp_properties.json` includes `${env:HOME}/Downloads/arduino*/hardware/**` and `libraries/**` (already configured).
- Verify `avr-gcc` path: `ls "$HOME/Downloads/arduino-1.8.19/hardware/tools/avr/bin/avr-gcc"`.
- If missing, install Arduino CLI and use its toolchain instead.

Testing compilation from the command line:

   # from workspace root
   arduino-cli compile --fqbn arduino:avr:mega arduino_mega/robot_navigation
   # then upload (if device attached):
   arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:mega arduino_mega/robot_navigation

Notes:
- The `.vscode/c_cpp_properties.json` included in this repo contains common locations; edit it if your system uses different paths.
- If using the official Arduino IDE (GUI), you don't need these steps for building; they only help VS Code IntelliSense.
