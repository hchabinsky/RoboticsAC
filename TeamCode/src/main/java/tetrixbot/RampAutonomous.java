
package tetrixbot;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PFollowLineController;

/**
 * Uses a PID Controller to follow a white line
 *
 * @author scottarmstrong, harrisonchabinsky
 */
@Autonomous(name="RampAutonomous", group="ElonDev")
// @Disabled
public class RampAutonomous extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    // define the robot hardware:
    HardwareTetrixBot robot   = new HardwareTetrixBot();   // Use the Drivebot hardware

    private double dt = 50.0;  // interval in milliseconds
    private double dT = dt / 1000.0;   // interval in seconds

    // parameters used by the controller:
    private double speed = 0.2;
    private double reference;
    private double Pc = 0.65;
    private double Kc = 0.01;
    private double Kp = 0.6 * Kc;
    private double Ki = (2 * Kp * dT) / Pc;
    private double Kd = (Kp * Pc) / (8 * dT);

    private double turnSpeed = 0.3;
    private int inchesToTarget = 17;
    private int loopCounter = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // initialize the hardware
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        int rampColor;
        int wiggleRoom = 5;
        String rampColorName = "";

        if (robot.sensorColor.blue() > robot.sensorColor.red()) {
            rampColor = robot.sensorColor.blue();
            rampColorName = "blue";
        } else {
            rampColor = robot.sensorColor.red();
            rampColorName = "red";
        }

        if (rampColorName.equals("blue")) {

            while (robot.sensorColor.blue() > rampColor - wiggleRoom) {
                robot.setRobotSpeed(-0.1);
            }

        } else if (rampColorName.equals("blue")) {

            while (robot.sensorColor.blue() > rampColor - wiggleRoom){
                robot.setRobotSpeed(-0.1);
            }

        }

        robot.stop();

        turnRobotAndCheckAlphaValues(turnSpeed, 180 * 2, PFollowLineController.TurnMethodActions.CHECKNONE, 0);

        reference = calibrateColorSensor();

        int encoderTarget = HardwareTetrixBot.convertInchesToTicks( Math.abs(inchesToTarget) );


        // displaying the reference value on the driver station
        telemetry.addData("Reference Value: ", reference);
        telemetry.update();

        Log.i("Kp: ", String.valueOf(Kp));

        sleep(2000);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // move the desired distance:
            robot.resetEncoderData();

            while (Math.abs(robot.motorLeft.getCurrentPosition()) < encoderTarget) {

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

            idle();
        }
        robot.stop();
    }

    public double calibrateColorSensor() throws InterruptedException {

        double calibrationReference = 0;
        int alphaMax = 0;
        int alphaMin = Integer.MAX_VALUE;

        alphaMax = turnRobotAndCheckAlphaValues(0.2, 90, PFollowLineController.TurnMethodActions.CHECKMAX, alphaMax);
        turnRobotAndCheckAlphaValues(0.2, -90, PFollowLineController.TurnMethodActions.CHECKNONE,0);
        alphaMin = turnRobotAndCheckAlphaValues(0.2, -90, PFollowLineController.TurnMethodActions.CHECKMIN, alphaMin);
        turnRobotAndCheckAlphaValues(0.2, 90, PFollowLineController.TurnMethodActions.CHECKNONE, 0);

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
    private int turnRobotAndCheckAlphaValues(double speed, double degrees, PFollowLineController.TurnMethodActions action, int alphaThreshold) throws InterruptedException {

        double direction = Math.signum(speed * degrees);

        if (direction == 0.0) {

            System.out.println("Speed or inches was 0");
        }

        int encoderTarget = HardwareTetrixBot.convertDegreesToTicks( Math.abs(degrees) );

        robot.resetEncoderData();
        robot.spin(Math.abs(speed) * direction);

        while (Math.abs(robot.motorLeft.getCurrentPosition()) < encoderTarget) {

            // only turns the robot if the specified action is CHECKNONE
            if (action != PFollowLineController.TurnMethodActions.CHECKNONE) {

                int currentAlpha = robot.sensorColor.alpha();

                // finds the maximum alpha value
                if (action == PFollowLineController.TurnMethodActions.CHECKMAX) {

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
