package org.firstinspires.ftc.teamcode.blockheads.learn;

/*
 * TankOdometry: keeps track of where the robot is (its x, its y, and which way it faces).
 *
 * "Odometry" means working out the robot's position from sensor readings while it drives,
 * the way a car's trip meter works out how far you have gone. We have two kinds of readings:
 *   - the drive-motor ENCODERS, which count how far each side's wheels have turned, and
 *   - the IMU (the gyro in the Control Hub), which says which way the robot is facing.
 *
 * Units and directions (they match the FTC SDK's IMU):
 *   - Distances are in millimetres (mm). Angles are in radians (180 degrees = pi radians).
 *   - x points the way the robot faced when we last called reset(). y points to the robot's LEFT.
 *   - Heading goes UP when the robot turns LEFT (counter-clockwise). Heading 0 = facing +x.
 *
 * This class has no FTC imports on purpose. It is plain Java math, so we can test it on a
 * laptop without a robot.
 */
public class TankOdometry {

    // How many encoder ticks equal 1 mm of driving, and the left-right distance between wheels.
    private final double ticksPerMm;
    private final double trackWidthMm;

    // The robot's pose (position + heading). x and y are in mm.
    private double x = 0;
    private double y = 0;
    private double heading = 0;          // from the IMU
    private double encoderHeading = 0;   // worked out from the wheels only (see update())

    // The readings from the last update, so we can see how much changed since then.
    private int lastLeftTicks = 0;
    private int lastRightTicks = 0;
    private double lastHeading = 0;

    public TankOdometry(double ticksPerMm, double trackWidthMm) {
        this.ticksPerMm = ticksPerMm;
        this.trackWidthMm = trackWidthMm;
    }

    // Put the robot at a known pose and remember the current encoder and IMU readings.
    // Call this once at the start (and any time you want to say "I am here now").
    public void reset(double xMm, double yMm, double headingRad, int leftTicks, int rightTicks) {
        x = xMm;
        y = yMm;
        heading = headingRad;
        encoderHeading = headingRad;
        lastLeftTicks = leftTicks;
        lastRightTicks = rightTicks;
        lastHeading = headingRad;
    }

    // Call this once every loop with the newest readings.
    public void update(int leftTicks, int rightTicks, double headingRad) {
        // 1) How far did each side's wheels roll since the last update? (ticks -> mm)
        double leftMm = (leftTicks - lastLeftTicks) / ticksPerMm;
        double rightMm = (rightTicks - lastRightTicks) / ticksPerMm;

        // 2) How far did the middle of the robot move? Just the average of the two sides.
        double forwardMm = (leftMm + rightMm) / 2.0;

        // 3) Which way were we pointing while we moved? During one loop the robot may turn a
        //    little, so we use the average of the heading before and after. This is a small
        //    trick that makes curved paths much more accurate.
        double turned = wrap(headingRad - lastHeading);
        double middleHeading = lastHeading + turned / 2.0;

        // 4) Move the robot's position that far in that direction (basic trigonometry:
        //    cos gives the x part of the move, sin gives the y part).
        x += forwardMm * Math.cos(middleHeading);
        y += forwardMm * Math.sin(middleHeading);

        // 5) The wheels can also tell us how much the robot turned: if the right side rolled
        //    farther than the left, the robot turned left. We keep this second opinion to
        //    compare with the IMU. When the wheels slip while turning, the two disagree, and
        //    that difference is how we measure the slip.
        encoderHeading += (rightMm - leftMm) / trackWidthMm;

        // Remember these readings for next time.
        heading = headingRad;
        lastHeading = headingRad;
        lastLeftTicks = leftTicks;
        lastRightTicks = rightTicks;
    }

    public double getXMm() { return x; }
    public double getYMm() { return y; }
    public double getHeadingRad() { return heading; }
    public double getEncoderHeadingRad() { return encoderHeading; }

    // Wrap an angle into the range -pi to +pi. Without this, going from 179 to -179 degrees
    // would look like a 358 degree turn instead of a 2 degree turn.
    public static double wrap(double angleRad) {
        while (angleRad > Math.PI) angleRad -= 2 * Math.PI;
        while (angleRad <= -Math.PI) angleRad += 2 * Math.PI;
        return angleRad;
    }
}
