# Remote Control Microscope (RCM)

The Remote Control Microscope (RCM) is an embedded system project that enables remote interaction with a microscope head capable of moving in 3D space, adjusting zoom levels, and rotating its field of view. It leverages a real-time operating system (FreeRTOS), hardware peripherals, and wireless communication for interactive control and feedback.

## System Overview

The RCM is a computer-controlled microscope that can:

- Move its stage along the **X, Y, and Z axes** in millimeters
- **Rotate** the viewed image by precise angles
- **Zoom in/out** on the target
- Be operated using onboard physical inputs or a **radio-controlled interface**
- Visually indicate current state and command status via onboard LEDs and displays
- Output current status through a **seven-segment display**
- Maintain responsiveness and modularity via **FreeRTOS task-based design**

## Architecture Highlights

- **Modular driver design** with reusable hardware abstraction layers
- **FreeRTOS-based control system** with proper task synchronization
- **NRF24L01+ radio module** for wireless control
- **State machine command input** via pushbuttons and switches
- **Hamming encoding** for reliable radio packet transmission
- **Live feedback system** for user interaction using RGB LEDs and seven-segment displays
- Expandable via optional modules for additional interfaces (e.g., IR remote, DAC output, CLI)

## Features

- **Radio Communication**: Wireless control using reliable, encoded packets
- **Command Interface**: Physical buttons allow cycling between movement, rotation, and zoom commands
- **System Feedback**:
  - Seven-segment display shows current X/Y/Z/Zoom/Rotation values
  - RGB LED and bar display reflect command mode
  - Diagnostic LEDs toggle on interaction events
- **Extensible Architecture**: Easily integrate new features using the existing mylib framework

## 📁 Project Structure

\`\`\`
repo/
│
├── pf/                 # Main application logic
│   ├── main.c          # System initialization only
│   ├── makefile
│   └── filelist.mk
│
└── mylib/              # Modular drivers and hardware control
    ├──header & driver files
\`\`\`

## Dependencies

- STM32 Nucleo-F429ZI development board
- NRF24L01+ radio transceiver module
- MFS board (LEDs, seven-segment, pushbuttons, switches)
- FreeRTOS
- Logic analyzer for debugging (optional)
- STM32 HAL or LL drivers (used within abstraction)

## Getting Started

1. Clone the repository.
2. Ensure toolchain and dependencies (ARM GCC, STM32CubeMX/HAL) are installed.
3. Configure `myconfig.h` with desired radio parameters.
4. Build using the provided `makefile`.
5. Flash to your STM32 Nucleo board.
6. Interact with the system using the pushbuttons or radio interface.

## 📡 Communication Protocol

All control packets are encoded using **Hamming codes** and transmitted in a fixed format. Each packet includes:

- A command type (JOIN, XYZ, ZOOM, ROT)
- A sender address (derived from unique ID)
- An ASCII-formatted payload string
- Padding to ensure fixed size before encoding

Packets are encoded to enhance robustness in noisy environments and decoded on the receiving end.



