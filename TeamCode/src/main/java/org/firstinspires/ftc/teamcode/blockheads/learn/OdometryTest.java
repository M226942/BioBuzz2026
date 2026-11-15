package org.firstinspires.ftc.teamcode.blockheads.learn;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * BH Odometry Test: drive the robot with the sticks and watch where it thinks it is.
 *
 * This is a TEST program. It does not run any mechanisms (no launcher or intake). It only
 * drives, and shows the robot's position (x, y, heading) from TankOdometry on the Driver
 * Station screen. Use it to learn how odometry behaves and to measure two numbers that the
 * team's autos currently guess: the real wheel distance and the real turning width.
 *
 * Do it in a free practice area. Keep a tape measure and a 2 metre (2000 mm) strip of tape.
 *
 * STEP 1 (INIT screen, nothing moves): push the robot FORWARD by hand. Both tick counts must go
 *   UP. Turn it LEFT by hand. Both "heading" numbers must go UP. If not, see the notes
 *   in LearnRobot.java (motor directions and hub mounting).
 * STEP 2 (distance check): press START, then A to zero. Push or drive the robot exactly
 *   2000 mm in a straight line. The "ticks per mm if you moved TAPE" line shows what the
 *   real number is. Compare it with the number in LearnRobot (about 1.78).
 * STEP 3 (turning width): press B, spin the robot in place by driving (right stick), at least
 *   one full turn, then press B again. It shows the REAL track width. Skid-steer robots usually
 *   give a bigger number than the measured one.
 * STEP 4 (drift): press A, drive around a square on the floor ending where you started, and
 *   see how far x and y are from 0. That error is what odometry drift looks like.
 *
 * Controls: left stick = forward/back, right stick = turn (same as the team TeleOp), A = zero the
 * position, B = start/stop the turning-width measurement.
 */
@TeleOp(name = "BH Odometry Test", group = "BH Learn")
public class OdometryTest extends OpMode {

    // The length of tape you use for the distance check, in mm.
    private static final double TAPE_MM = 2000;
    // Slow the driving down so the test is safe and easy to control.
    private static final double DRIVE_SCALE = 0.6;

    private final LearnRobot robot = new LearnRobot();
    private final TankOdometry odometry =
            new TankOdometry(LearnRobot.TICKS_PER_MM, LearnRobot.TRACK_WIDTH_MM);

    // Ticks at the moment we pressed A, for the distance check.
    private int zeroLeft = 0;
    private int zeroRight = 0;

    // The turning-width measurement: adds up how far the wheels rolled and how far the robot
    // turned (from the IMU) while the measurement is on.
    private boolean measuring = false;
    private double wheelMmSum = 0;
    private double turnedRadSum = 0;
    private int prevLeft = 0;
    private int prevRight = 0;
    private double prevHeading = 0;
    private double measuredTrackMm = 0;

    // So a button press only counts once, not on every loop.
    private boolean aWasDown = false;
    private boolean bWasDown = false;

    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addLine("Push the robot FORWARD by hand: both tick numbers must go UP.");
        telemetry.addLine("Turn it LEFT by hand: both heading numbers must go UP.");
        telemetry.update();
    }

    // init_loop() runs again and again after INIT, before START. We use it for the hand check.
    @Override
    public void init_loop() {
        telemetry.addLine("HAND CHECK (nothing moves). Push the robot around by hand.");
        telemetry.addData("Ticks left / right", "%d / %d", robot.leftTicks(), robot.rightTicks());
        telemetry.addData("Heading (IMU)", "%.1f deg", Math.toDegrees(robot.headingRad()));
        telemetry.addLine("Forward -> both ticks UP.  Turn left -> heading UP.");
        telemetry.update();
    }

    @Override
    public void start() {
        zeroPose();
    }

    @Override
    public void loop() {
        // 1) Drive with the sticks (slowed down).
        double forward = -gamepad1.left_stick_y * DRIVE_SCALE;
        double rotate = gamepad1.right_stick_x * DRIVE_SCALE;
        robot.drive(forward, rotate);

        // 2) Read the sensors and update the odometry.
        int left = robot.leftTicks();
        int right = robot.rightTicks();
        double heading = robot.headingRad();
        odometry.update(left, right, heading);

        // 3) Buttons. A zeroes the pose, B starts and stops the turning-width measurement.
        if (gamepad1.a && !aWasDown) {
            zeroPose();
        }
        aWasDown = gamepad1.a;

        if (gamepad1.b && !bWasDown) {
            measuring = !measuring;
            wheelMmSum = 0;
            turnedRadSum = 0;
        }
        bWasDown = gamepad1.b;

        // 4) While measuring: each wheel rolls (turn angle x track width / 2) mm during a spin
        //    in place, so (both wheels added together) / (turn angle) = track width.
        if (measuring) {
            wheelMmSum += (Math.abs(left - prevLeft) + Math.abs(right - prevRight)) / LearnRobot.TICKS_PER_MM;
            turnedRadSum += Math.abs(TankOdometry.wrap(heading - prevHeading));
            if (turnedRadSum > Math.PI) {   // wait until we have at least a half turn
                measuredTrackMm = wheelMmSum / turnedRadSum;
            }
        }
        prevLeft = left;
        prevRight = right;
        prevHeading = heading;

        // 5) Show everything.
        telemetry.addData("x, y (mm)", "%.0f, %.0f", odometry.getXMm(), odometry.getYMm());
        telemetry.addData("x, y (in)", "%.1f, %.1f", odometry.getXMm() / 25.4, odometry.getYMm() / 25.4);
        telemetry.addData("Heading from IMU", "%.1f deg", Math.toDegrees(odometry.getHeadingRad()));
        telemetry.addData("Heading from wheels", "%.1f deg", Math.toDegrees(odometry.getEncoderHeadingRad()));
        telemetry.addData("IMU minus wheels", "%.1f deg  (wheel slip)",
                Math.toDegrees(TankOdometry.wrap(odometry.getHeadingRad() - odometry.getEncoderHeadingRad())));
        telemetry.addData("Ticks left / right", "%d / %d", left, right);

        double avgTicks = ((left - zeroLeft) + (right - zeroRight)) / 2.0;
        telemetry.addData("Ticks per mm if you moved TAPE", "%.3f  (code uses %.3f)",
                Math.abs(avgTicks) / TAPE_MM, LearnRobot.TICKS_PER_MM);
        telemetry.addData("Turning width", measuring
                ? String.format("measuring... turned %.0f deg, %.0f mm so far",
                        Math.toDegrees(turnedRadSum), measuredTrackMm)
                : (measuredTrackMm > 0 ? String.format("%.0f mm  (code uses %.0f)", measuredTrackMm,
                        LearnRobot.TRACK_WIDTH_MM) : "press B, spin, press B"));
        telemetry.addData("Battery", "%.2f V", robot.batteryVolts());
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }

    // Zero the pose here: x = 0, y = 0, heading = 0, and remember the tick counts.
    private void zeroPose() {
        robot.resetHeading();
        zeroLeft = robot.leftTicks();
        zeroRight = robot.rightTicks();
        odometry.reset(0, 0, robot.headingRad(), zeroLeft, zeroRight);
        prevLeft = zeroLeft;
        prevRight = zeroRight;
        prevHeading = robot.headingRad();
    }
}
