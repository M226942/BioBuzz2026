# Odometry and heading: first steps (new files only)

These files add three TEST programs to the Driver Station. They do not change any existing team code.
They drive the 6WD StarterBot only (no launcher, intake or servos).

| Program (group "BH Learn") | What it does |
|---|---|
| **BH Odometry Test** (TeleOp) | Shows the robot's position (x, y, heading). Measures the real wheel distance and the real turning width. |
| **BH Heading Hold Drive** (TeleOp) | Your arcade drive, plus "let go of the turn stick and the robot keeps facing the same way". |
| **BH IMU Turn Test** (Autonomous) | Turns to +90, 0, -90, 0 degrees using the IMU, and shows how many degrees off each turn ended. |

The math lives in two small plain-Java classes you can read in about 10 minutes each:
`TankOdometry.java` (position from the wheel encoders + IMU) and `HeadingHold.java` (the P controller).
`LearnRobot.java` holds the motor names, directions and the IMU setup in one place.

## 15-minute practice plan

1. **BH Odometry Test, INIT screen, nothing moves.** Push the robot FORWARD by hand: both tick numbers must go UP.
   Turn it LEFT by hand: the heading must go UP. If a number goes the wrong way, flip that line in `LearnRobot.java`
   (motor direction or hub mounting). Do this first, every time.
2. **Distance.** Press START, then A. Push or drive exactly 2000 mm (tape). Read "Ticks per mm if you moved TAPE".
   The code uses 1.78. If yours is different, the wheel size or the tape was off.
3. **Turning width.** Press B, spin the robot in place at least one full turn with the right stick, press B again.
   The code uses 402 mm. Expect a bigger number: the six wheels scrub sideways when the robot turns.
4. **Drift.** Press A, drive a square on the floor back to the start mark, and read x and y. Anything over about 50 mm
   is odometry drift (wheel slip). That is what a Pinpoint or dead wheels would fix.
5. **BH IMU Turn Test.** Run it. Write down the four "ended X deg off" numbers.
6. **BH Heading Hold Drive.** Drive forward, let go of the turn stick, bump the robot, and feel it turn back.
   Press B to compare with hold off.

Write down: ticks per mm, turning width, drift after the square, the four turn errors, and anything that felt wrong.
Those numbers decide what to build next (drive-to-point autos, then shot and aim improvements).

## How odometry works, in five lines

1. Each loop, read how far the left and right wheels rolled (ticks to mm).
2. The middle of the robot moved the average of the two.
3. Use the IMU for which way the robot was pointing (the wheels slip when turning, the IMU does not).
4. Move the position that far in that direction: `x += d * cos(heading)`, `y += d * sin(heading)`.
5. Repeat every loop (about 50 times a second). The small errors add up, which is called drift.

## Tuning the heading hold

The numbers in `HeadingHold.java` (kP, kS, kD, maxPower) are STARTING values and were never tested on a robot.
The steps are at the bottom of that file. Change one number at a time.

## What was tested

- The math classes were tested on a laptop: straight lines, curves, crossing the 180 degree line, spins with wheel slip,
  and a simulated 90 degree turn at two battery strengths.
- The three programs were compiled against SDK 12 and run on a pretend robot, so there are no crashes in the
  programs themselves: the four turns finish and the position follows the pretend robot.
- NOT tested: the real robot. The motor directions (both REVERSE, as in the team's current TeleOp), the hub mounting
  and the heading hold gains must be checked on the robot with step 1 and the IMU Turn Test.
