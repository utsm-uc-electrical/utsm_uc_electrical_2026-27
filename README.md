# utsm_uc_electrical_2026-27
The Electrical Division codebase for the 2026–2027 Urban Concept vehicle, developed as part of the University of Toronto Supermileage Design Team.

The [LoRa USB receiver](receiver/README.md) decodes `transmit.ino` packets on a second ESP32 and streams telemetry to a computer over USB serial. Open `receiver/receiver.ino` as a separate Arduino sketch.
