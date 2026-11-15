package org.firstinspires.ftc.teamcode.blockheads.learn;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * LearnRobot: the drive motors, the IMU and the battery sensor in ONE place.
 *
 * The three test programs in this folder (BH Odometry Test, BH Heading Hold Drive and
 * BH IMU Turn Test) all use this class, so the motor names and directions are written once.
 * That is a good habit for the whole team code: when a motor direction is wrong you fix one
 * line, not four files.
 *
 * It only sets up the DRIVE. It does not touch the launcher, intake or servos.
 */
public class LearnRobot {

    // Names in the robot configuration (same as the team's code).
    public static final String LEFT_DRIVE_NAME = "left_drive";
    public static final String RIGHT_DRIVE_NAME = "right_drive";

    // Directions. These copy what the team's TeleOp on master does today (both REVERSE), which
    // drives correctly on the real robot. The INIT screen of "BH Odometry Test" checks them:
    // pushing the robot FORWARD by hand must make BOTH tick counts go UP.
    // If a count goes DOWN, flip that side here.
    public static final DcMotorSimple.Direction LEFT_DRIVE_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction RIGHT_DRIVE_DIRECTION = DcMotorSimple.Direction.REVERSE;

    // How the Control Hub is mounted (checked on the robot 2026-10-19): it stands on its side,
    // the REV logo faces the robot's RIGHT and the USB ports face the BACK (the launcher end).
    // If the heading goes the wrong way or does not change, these two are wrong.
    public static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO =
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB =
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

    // Wheel numbers, copied from the team's autos. 537.7 ticks = one turn of the 312 RPM
    // goBILDA motor's output shaft. A wheel turn rolls the robot one wheel circumference.
    public static final double WHEEL_DIAMETER_MM = 96;
    public static final double TICKS_PER_REV = 537.7;
    public static final double TICKS_PER_MM = TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI);
    // Distance between the left and right wheels. A skid-steer robot scrubs its wheels sideways
    // when it turns, so the REAL turning width is usually bigger than the measured one.
    // BH Odometry Test measures the real one (press B and spin the robot).
    public static final double TRACK_WIDTH_MM = 402;

    public DcMotorEx leftDrive = null;
    public DcMotorEx rightDrive = null;
    public IMU imu = null;
    private VoltageSensor battery = null;

    public void init(HardwareMap hardwareMap) {
        leftDrive = hardwareMap.get(DcMotorEx.class, LEFT_DRIVE_NAME);
        rightDrive = hardwareMap.get(DcMotorEx.class, RIGHT_DRIVE_NAME);
        leftDrive.setDirection(LEFT_DRIVE_DIRECTION);
        rightDrive.setDirection(RIGHT_DRIVE_DIRECTION);

        // BRAKE makes the wheels resist being pushed when the power is 0.
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set the encoder counts back to 0, then run the motors with plain power commands.
        // We still READ the encoders for odometry; we just do not ask the hub to control speed.
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // The IMU is told how the hub is mounted, so "heading" means how the ROBOT is turned.
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(HUB_LOGO, HUB_USB)));
        imu.resetYaw();

        // The battery voltage sensor is built into the hub.
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            battery = sensor;
            break;
        }
    }

    // Drive the way the team's arcadeDrive does: forward is -1..1, rotate is -1..1 and
    // POSITIVE rotate turns the robot RIGHT (clockwise).
    public void drive(double forward, double rotate) {
        double left = forward + rotate;
        double right = forward - rotate;
        // If one side would go over full power, shrink both so the robot still turns the same way.
        double biggest = Math.max(1.0, Math.max(Math.abs(left), Math.abs(right)));
        leftDrive.setPower(left / biggest);
        rightDrive.setPower(right / biggest);
    }

    public void stop() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public int leftTicks() { return leftDrive.getCurrentPosition(); }
    public int rightTicks() { return rightDrive.getCurrentPosition(); }

    // Heading in radians. Goes UP when the robot turns LEFT. 0 = where it faced at resetYaw().
    public double headingRad() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void resetHeading() {
        imu.resetYaw();
    }

    public double batteryVolts() {
        return battery == null ? 0 : battery.getVoltage();
    }
}
