## Demo 2: Single Marker Scanning and Movement

This directory contains files and subdirectories pertaining to the first system-wide integration of the robot subsystems for Demo 2, with the objective of turning towards a marker at an unknown location, calculating the distance away from it, and driving to its location. For all script types, this also necessitated the creation of a finite state machine architecture.

OpenCV scripts mostly pertain to preliminary tests and implementations of the distance calculation mentioned above, in addition to the angle calculations and other video-feed based script segments found in previous demonstrations.

Arduino scripts mostly pertain to implementing prior motion and motor communication code with the aforementioned state machine, in addition to communicating with the Raspberry Pi for directions based off of a series of I2C signals and GPIO lines.
