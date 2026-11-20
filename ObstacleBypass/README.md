# Obstacle Bypass (ESP32)

`ObstacleBypass.ino` drives the robot straight, goes around anything blocking
its path, and returns to the original line and heading. See the header comment
in the sketch for the full state machine.

## Ultrasonic sensor orientation

The side sensors are **not** mounted at 90 degrees to the direction of travel.
They point diagonally forward, 45 degrees off the front sensor.

Angles use the unit circle, with the robot's forward direction at pi/2:

| Sensor | Unit-circle angle | Degrees | Relative to forward |
|--------|-------------------|---------|---------------------|
| Front  | pi/2              | 90      | 0 (straight ahead)  |
| Left   | 3pi/4             | 135     | 45 to the left      |
| Right  | pi/4              | 45      | 45 to the right     |
| Back   | -pi/2             | -90     | 180 (straight back) |

```
                 Front (pi/2)
                      ^
     Left (3pi/4)     |     Right (pi/4)
                \     |     /
                 \    |    /
                  +-------+
                  |       |
                  | ROBOT |
                  |       |
                  +-------+
                      |
                      v
                 Back (-pi/2)
```

### What this means for readings

A side sensor reading `d` is measured along the 45 degree line, not straight
out to the side. It splits into:

- distance straight out to the side: `d * sin(45°) ≈ 0.707 * d`
- distance ahead of the sensor: `d * cos(45°) ≈ 0.707 * d`

So a side sensor sees objects that are ahead of the robot as well as beside
it, and its raw reading is about 1.41 times the true sideways gap to a wall
running parallel to the robot.

## Pinout

### Ultrasonic sensors

| Sensor | Trig pin | Echo pin |
|--------|----------|----------|
| Front  | GPIO 23  | GPIO 22  |
| Left   | GPIO 18  | GPIO 19  |
| Right  | GPIO 17  | GPIO 5   |
| Back   | GPIO 4   | GPIO 16  |

Sensors are read one at a time with a 30 ms gap so one sensor's echo isn't
picked up by the next. A read with no echo within 30 ms returns 400 cm.

### Motors (HB-25 motor controllers)

| Motor | Signal pin |
|-------|------------|
| Left  | GPIO 33    |
| Right | GPIO 32    |

The HB-25 is controlled with servo-style pulses (one pulse about every 20 ms).
The pulse width sets direction and speed: 1500 us is stop, and each 5 us away
from 1500 is 1% of the driver's output, up to 100% at 1000 us or 2000 us. The
robot only uses the three values below, and they are constant: forward and
reverse are always 30% of the driver's output (150 us from stop).

| Command | Pulse width | Output |
|---------|-------------|--------|
| Forward | 1350 us     | 30% forward  |
| Stop    | 1500 us     | 0% (neutral) |
| Reverse | 1650 us     | 30% reverse  |

Because of how the motors are wired, pulses **below** 1500 us drive the robot
forward and pulses above 1500 us drive it backward.

In the sketch these are `MOTOR_FORWARD_US`, `MOTOR_STOP_US` and
`MOTOR_REVERSE_US`. The servo outputs are attached with a 1000-2000 us range,
so no pulse can go outside the driver's limits.

A pivot turn drives one wheel forward and the other in reverse, both at 30%.
The only time a pulse is not one of these three values is the straight-line
correction in `driveStraight()`, which shifts each wheel's forward pulse by up
to 60 us (slowing the wheel that is ahead and speeding up the one behind) to
keep the robot driving straight.

### Wheel encoders

| Encoder | Channel A | Channel B |
|---------|-----------|-----------|
| Left    | GPIO 25   | GPIO 26   |
| Right   | GPIO 27   | GPIO 14   |

Both channels use internal pull-ups. Interrupts are attached to channel A only
(on change); channel B gives the direction.

- Encoder resolution: 36 pulses per wheel revolution
- Wheel diameter: 15.24 cm (6 in)

## Robot dimensions

Set these in the sketch under `ROBOT DIMENSIONS` after measuring the robot:

| Constant          | Current value | Meaning                                   |
|-------------------|---------------|-------------------------------------------|
| `TRACK_WIDTH_CM`  | 38.0          | Center of left wheel to center of right   |
| `ROBOT_WIDTH_CM`  | 40.0          | Outer width, including anything sticking out |
| `ROBOT_LENGTH_CM` | 45.0          | Outer length                              |

### Measured on the robot

| Measurement                      | Value   |
|----------------------------------|---------|
| Front sensor to back sensor      | 46.5 cm |
