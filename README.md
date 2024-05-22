# Solar Tracker

This Arduino project utilizes servo motors to create a control system based on Light Dependent Resistors (LDRs) input. The system is designed to adjust the angles of two servo motors, one for the upper direction (θ) and the other for the base direction (φ), in response to environmental light changes detected by the LDRs.

## Setup

### Components Required

- Arduino board (e.g., Arduino Uno)
- Servo motors (2)
- Light Dependent Resistors (LDRs) (4)
- Jumper wires

### Connections

- Connect the LDRs to analog pins A0 to A3 on the Arduino board.
- Attach the servo motors to digital pins 10 and 3 for the upper and base directions, respectively.

## Installation

1. Clone or download the project repository.
2. Open the Arduino IDE.
3. Import the project files into the IDE.
4. Upload the code to your Arduino board.

## Usage

1. Power up the Arduino board.
2. The system will automatically read the LDR values and adjust the servo angles accordingly.
3. The serial monitor can be used to debug and monitor the LDR readings.

## Configuration

- Adjust the `Angle_Incrementation` constant to change the increment value for servo movement.
- Modify the `error` constant to set the error threshold for LDR readings.
- Update the `delayy` constant to adjust the delay between servo movements.


## Side Note:
I wrote this file using the aide of AI:
- ChatGPT
- OpenAI
- [Website](https://www.openai.com)
