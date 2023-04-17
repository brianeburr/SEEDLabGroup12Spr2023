## Final Demo: Multi-Marker Motion

The main additional function of the final demo is integrating a looping system into the pi-side state machine to allow it to step through multiple markers in its path.
This mainly boils down to including a variable that tracks the current marker that it should be searching for, as well as including extra logic to ensure the robot only tracks and approaches the current marker in the sequence.
