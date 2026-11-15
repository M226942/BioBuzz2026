package org.firstinspires.ftc.teamcode.blockheads.learn;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * BH IMU Turn Test: the robot turns to exact angles using the IMU instead of timers.
 *
 * It faces its starting direction, then turns to +90 degrees (left), back to 0, to -90 degrees
 * (right) and back to 0, pausing 1 second between turns. After each turn the Driver Station
 * shows how many degrees off the robot ended up. Compare with the team's timed turns
 * (for example "power 0.75 for 0.2 seconds"): those turn a different amount when the battery
 * or the floor changes. An IMU turn does not care.
 *
 * Put the robot in a free practice area with room to spin. It only turns; it does not drive away.
 * If a turn takes more than 4 seconds the robot stops and says so (a safety stop).
 *
 * This is a state machine, the same idea as the team autos: a list of steps (the enum), and each
 * loop runs the code for the current step and moves on when the step is done.
 */
@Autonomous(name = "BH IMU Turn Test", group = "BH Learn")
public class ImuTurnTest extends OpMode {

    private enum Step { WAIT, TURNING, PAUSE, DONE }

    // The angles to turn to, in degrees. Positive = left, negative = right.
    private static final double[] TARGETS_DEG = {90, 0, -90, 0};
    private static final double DONE_ERROR_DEG = 2.0;     // "close enough" for a finished turn
    private static final double SETTLE_SECONDS = 0.4;     // must stay close this long
    private static final double TURN_TIMEOUT_SECONDS = 4.0;
    private static final double PAUSE_SECONDS = 1.0;

    private final LearnRobot robot = new LearnRobot();
    private final HeadingHold hold = new HeadingHold();
    private final ElapsedTime stepTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime settleTimer = new ElapsedTime();

    private Step step = Step.WAIT;
    private int turnIndex = 0;
    private int turnsDone = 0;     // how many turns have finished (for the results list)
    private final double[] finalErrorDeg = new double[TARGETS_DEG.length];
    private String problem = "";

    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addLine("Ready. Press START and keep clear: the robot will spin in place.");
        telemetry.update();
    }

    @Override
    public void start() {
        robot.resetHeading();     // the way the robot faces now is 0 degrees
        stepTimer.reset();
        loopTimer.reset();
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        double heading = robot.headingRad();

        switch (step) {
            case WAIT:
                robot.stop();
                if (stepTimer.seconds() > 1.0) {
                    startTurn();
                }
                break;

            case TURNING:
                // Ask the controller how hard to turn toward the target, and drive that.
                robot.drive(0, hold.rotateCommand(heading, dt));

                // The turn is finished when we have stayed within 2 degrees for 0.4 seconds.
                if (Math.abs(Math.toDegrees(hold.error(heading))) > DONE_ERROR_DEG) {
                    settleTimer.reset();
                }
                if (settleTimer.seconds() > SETTLE_SECONDS) {
                    finishTurn(heading);
                } else if (stepTimer.seconds() > TURN_TIMEOUT_SECONDS) {
                    problem = "Turn " + (turnIndex + 1) + " timed out. Check hub mounting / gains.";
                    finishTurn(heading);
                }
                break;

            case PAUSE:
                robot.stop();
                if (stepTimer.seconds() > PAUSE_SECONDS) {
                    turnIndex++;
                    if (turnIndex < TARGETS_DEG.length) {
                        startTurn();
                    } else {
                        step = Step.DONE;
                    }
                }
                break;

            case DONE:
                robot.stop();
                break;
        }

        telemetry.addData("Step", step);
        telemetry.addData("Heading", "%.1f deg", Math.toDegrees(heading));
        if (step == Step.TURNING) {
            telemetry.addData("Turning to", "%.0f deg (error %.1f)", TARGETS_DEG[turnIndex],
                    Math.toDegrees(hold.error(heading)));
        }
        for (int i = 0; i < turnsDone; i++) {
            telemetry.addData("Turn " + (i + 1) + " to " + (int) TARGETS_DEG[i],
                    "ended %.1f deg off", finalErrorDeg[i]);
        }
        if (!problem.isEmpty()) {
            telemetry.addLine(problem);
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }

    private void startTurn() {
        hold.setTarget(Math.toRadians(TARGETS_DEG[turnIndex]));
        settleTimer.reset();
        stepTimer.reset();
        step = Step.TURNING;
    }

    private void finishTurn(double heading) {
        robot.stop();
        finalErrorDeg[turnIndex] = Math.toDegrees(hold.error(heading));
        turnsDone++;
        stepTimer.reset();
        step = Step.PAUSE;
    }
}
