
package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Uses a P Controller to follow a white line
 *
 * @author scottarmstrong, harrisonchabinsky
 */
@Autonomous(name="PFollowerLineController", group="ElonDev")
public class PFollowLineController extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    // define the robot hardware:
    HardwareDriveBot robot   = new HardwareDriveBot();   // Use the Drivebot hardware

    private double dt = 50.0;  // interval in milliseconds

    // parameters used by the controller:
    private double speed = 0.2;
    private double reference;
    private double Kc = 0.01;
    private double Kp = 0.5 * Kc;

    private int loopCounter = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // initialize the hardware
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        reference = calibrateColorSensor();

        // displaying the reference value on the driver station
        telemetry.addData("Reference Value: ", reference);
        telemetry.update();

        Log.i("Kp: ", String.valueOf(Kp));

        sleep(2000);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            int brightness = robot.sensorColor.alpha();

            double error = reference - brightness;

            double turn = Kp * error;

            robot.motorLeft.setPower(speed - turn);
            robot.motorRight.setPower(speed + turn);

            loopCounter++;
            double nextTimeSlot = loopCounter * dt;

            while (runtime.milliseconds() < nextTimeSlot) {
                idle();
            }
        }

        robot.stop();
    }

    public double calibrateColorSensor() throws InterruptedException {

        double calibrationReference = 0;
        int alphaMax = 0;
        int alphaMin = Integer.MAX_VALUE;

        alphaMax = turnRobotAndCheckAlphaValues(0.2, 90, TurnMethodActions.CHECKMAX, alphaMax);
        turnRobotAndCheckAlphaValues(0.2, -90, TurnMethodActions.CHECKNONE,0);
        alphaMin = turnRobotAndCheckAlphaValues(0.2, -90, TurnMethodActions.CHECKMIN, alphaMin);
        turnRobotAndCheckAlphaValues(0.2, 90, TurnMethodActions.CHECKNONE, 0);

        calibrationReference = (alphaMax + alphaMin) / 2;

        // print process parameters in console:
        telemetry.addData("Reference Value: ", calibrationReference);
        telemetry.update();
        System.out.println("Error: " + (calibrationReference - robot.sensorColor.alpha()));

        return calibrationReference;
    }

    public enum TurnMethodActions {
        CHECKMAX, CHECKMIN, CHECKNONE;
    }

    /**
     * turns the robot a given number of degrees at a given speed and checks alpha thresholds
     *
     * @param speed    robot speed between -1.0 ... 1.0
     * @param degrees  turn angle in degrees
     * @param action   determines which alpha threshold, if any, will be checked
     * @param alphaThreshold min or max alpha value to update
     * @throws InterruptedException
     */
    private int turnRobotAndCheckAlphaValues(double speed, double degrees, TurnMethodActions action, int alphaThreshold) throws InterruptedException {

        double direction = Math.signum(speed * degrees);

        if (direction == 0.0) {

            System.out.println("Speed or inches was 0");
        }

        int encoderTarget = HardwareDriveBot.convertDegreesToTicks( Math.abs(degrees) );

        robot.resetEncoderData();
        robot.spin(Math.abs(speed) * direction);

        while (Math.abs(robot.motorLeft.getCurrentPosition()) < encoderTarget) {

            // only turns the robot if the specified action is CHECKNONE
            if (action != TurnMethodActions.CHECKNONE) {

                int currentAlpha = robot.sensorColor.alpha();

                // finds the maximum alpha value
                if (action == TurnMethodActions.CHECKMAX) {

                    if (currentAlpha > alphaThreshold) {

                        alphaThreshold = currentAlpha;
                    }

                    // finds the minimum alpha value
                } else {

                    if (currentAlpha < alphaThreshold) {

                        alphaThreshold = currentAlpha;
                    }
                }
            }

            idle();
        }
        robot.stop();

        return alphaThreshold;
    }
}
