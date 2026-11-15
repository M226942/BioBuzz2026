package org.firstinspires.ftc.teamcode.blockheads.learn;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * BH Heading Hold Drive: the team's arcade drive plus "heading hold".
 *
 * Heading hold means: when you let go of the turn stick, the robot keeps facing the way it was
 * facing. Without it, a six-wheel skid-steer robot drifts sideways a little on every drive.
 * It also helps when another robot bumps you: the robot turns itself back.
 *
 * Controls (same as the team TeleOp for driving):
 *   left stick up/down = forward/back
 *   right stick left/right = turn (while you hold it, the robot turns freely)
 *   B = turn heading hold on or off (so you can compare how it feels)
 *   X = "face this way now": makes the current direction the new target
 *
 * This is a drive-only test. It does not run the launcher or intake. To use it in the real
 * TeleOp, it is three lines (see the bottom of this file).
 */
@TeleOp(name = "BH Heading Hold Drive", group = "BH Learn")
public class HeadingHoldTeleOp extends OpMode {

    private final LearnRobot robot = new LearnRobot();
    private final HeadingHold hold = new HeadingHold();
    private final ElapsedTime loopTimer = new ElapsedTime();

    private boolean holdOn = true;
    private boolean bWasDown = false;

    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addLine("Ready. Drive with the sticks. B = heading hold on/off.");
        telemetry.update();
    }

    @Override
    public void start() {
        // Whatever way the robot faces now is "heading 0" and the first target.
        robot.resetHeading();
        hold.setTarget(0);
        loopTimer.reset();
    }

    @Override
    public void loop() {
        // How much time passed since the last loop. The "brake" part of the hold needs it.
        double dt = loopTimer.seconds();
        loopTimer.reset();

        double heading = robot.headingRad();
        double forward = -gamepad1.left_stick_y;
        double stick = gamepad1.right_stick_x;

        // B flips heading hold on and off (counted once per press).
        if (gamepad1.b && !bWasDown) {
            holdOn = !holdOn;
            hold.setTarget(heading);
        }
        bWasDown = gamepad1.b;

        // X says "hold THIS direction from now on".
        if (gamepad1.x) {
            hold.setTarget(heading);
        }

        // With the hold on, update() decides the turn power. With it off, the stick is used as is.
        double rotate = holdOn ? hold.update(heading, stick, dt) : stick;
        robot.drive(forward, rotate);

        telemetry.addData("Heading hold", holdOn ? "ON" : "OFF");
        telemetry.addData("Heading", "%.1f deg", Math.toDegrees(heading));
        telemetry.addData("Target", "%.1f deg", Math.toDegrees(hold.getTarget()));
        telemetry.addData("Error", "%.1f deg", Math.toDegrees(hold.error(heading)));
        telemetry.addData("Turn power", "%.2f", rotate);
        telemetry.addData("Battery", "%.2f V", robot.batteryVolts());
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}

/*
 * TO ADD HEADING HOLD TO THE REAL TELEOP (BioBuzzStarterBotTeleop):
 *   1. Make the IMU like LearnRobot does, and add:  HeadingHold hold = new HeadingHold();
 *   2. In loop(), find the line  arcadeDrive(-gamepad1.left_stick_y, gamepad1.right_stick_x);
 *   3. Change it to:
 *        arcadeDrive(-gamepad1.left_stick_y,
 *                    hold.update(headingRad, gamepad1.right_stick_x, dtSeconds));
 *      where headingRad is the IMU yaw in radians and dtSeconds is the time since the last loop.
 * Test it with this program first, so you know the gains work on the real robot.
 */
