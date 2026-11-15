package org.firstinspires.ftc.teamcode.blockheads.learn;

/*
 * HeadingHold: makes the robot keep facing the same way, using the IMU heading.
 *
 * Two uses:
 *   - In TeleOp: when you let go of the turn stick, the robot holds the direction it was
 *     facing instead of drifting. Call update() every loop.
 *   - In autonomous: to turn to an exact angle, call setTarget(angle), then call
 *     rotateCommand() every loop until the error is small.
 *
 * It uses a "P controller" (the P in PID). The idea: the farther the robot is from where it
 * should face, the harder it turns back. Close to the target it turns gently.
 *
 * Plain Java math, no FTC imports, so it can be tested on a laptop.
 *
 * IMPORTANT: these gains (kP, kS, kD, maxPower) are STARTING numbers. They must be tuned on the
 * real robot. The tuning steps are in the comments at the bottom of this file.
 *
 * Direction convention: this matches the team's arcadeDrive(forward, rotate) where
 *   leftPower = forward + rotate and rightPower = forward - rotate.
 * A POSITIVE rotate value turns the robot RIGHT (clockwise). The IMU heading goes UP when the
 * robot turns LEFT. That is why the answer below is flipped with a minus sign.
 */
public class HeadingHold {

    public double kP = 1.2;          // power per radian of error (1 radian is about 57 degrees)
    public double kS = 0.07;         // a small extra push, because a 6-wheel robot will not
                                     // start turning on really tiny power
    public double kD = 0.05;         // brakes the turn: opposes how fast the robot is turning
    public double maxPower = 0.5;    // never turn faster than this while holding
    public double deadbandRad = Math.toRadians(1.5);   // close enough: stop correcting
    public double stickDeadband = 0.05;                // below this the stick counts as "let go"

    private double targetHeading = 0;
    private double lastHeading = 0;
    private boolean haveLast = false;

    public void setTarget(double headingRad) {
        targetHeading = headingRad;
    }

    public double getTarget() {
        return targetHeading;
    }

    // Error = how far we still have to turn. Positive means the target is to our LEFT.
    public double error(double headingRad) {
        return TankOdometry.wrap(targetHeading - headingRad);
    }

    // The rotate command (-1 to 1) that turns the robot toward the target.
    // dtSeconds is the time since the last call (used for the braking part).
    public double rotateCommand(double headingRad, double dtSeconds) {
        double error = error(headingRad);

        // How fast are we turning right now? (change in heading divided by time)
        double rate = 0;
        if (haveLast && dtSeconds > 1e-4) {
            rate = TankOdometry.wrap(headingRad - lastHeading) / dtSeconds;
        }
        lastHeading = headingRad;
        haveLast = true;

        // Close enough: do nothing, so the robot does not twitch back and forth.
        if (Math.abs(error) < deadbandRad) {
            return 0;
        }

        // The controller: push toward the target (kP * error), add the small starting push
        // (kS), and subtract a brake that grows with the turning speed (kD * rate).
        double leftTurnPower = kP * error + kS * Math.signum(error) - kD * rate;
        leftTurnPower = Math.max(-maxPower, Math.min(maxPower, leftTurnPower));

        // Positive leftTurnPower means "turn left". The team's rotate is positive for RIGHT.
        return -leftTurnPower;
    }

    // The TeleOp version. stick is the driver's turn stick (positive = turn right).
    // While the driver is turning, we just follow: the target keeps updating to wherever the
    // robot points. When the driver lets go, the target freezes and the robot holds it.
    public double update(double headingRad, double stick, double dtSeconds) {
        if (Math.abs(stick) > stickDeadband) {
            targetHeading = headingRad;
            lastHeading = headingRad;
            haveLast = true;
            return stick;
        }
        return rotateCommand(headingRad, dtSeconds);
    }
}

/*
 * HOW TO TUNE (do this on the robot, in a practice area, with the robot on the floor):
 *  1. Start with the numbers above. Run "BH IMU Turn Test". It turns to +90, 0, -90, 0 degrees.
 *  2. If it turns the WRONG way, the hub mounting (HUB_LOGO / HUB_USB in LearnRobot) is wrong.
 *  3. If it creeps slowly and stops short, raise kS a little (0.07 -> 0.09).
 *  4. If it swings past the target and wobbles, lower kP or raise kD a little.
 *  5. If it twitches back and forth at the end, make deadbandRad a bit bigger.
 *  Change one number at a time and write down what happened.
 */
