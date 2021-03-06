package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * MoveWithEncoder moves robot through an obsticle course
 *
 * @author scottarmstrong, harrisonchabinsky
 */

@Autonomous(name="Move With Encoders", group="Elon")
public class MoveWithEncoder extends LinearOpMode {

    // define the robot hardware
    HardwareDriveBot robot = new HardwareDriveBot();

    @Override
    public void runOpMode() throws InterruptedException {

        // initialize the robot
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        updateEncoderData(robot.motorLeft.getCurrentPosition());

        double moveRobotSpeed = 0.5;

        double turnRobotSpeed = 0.3;

        int pauseTimeBetweenMovements = 250;

        sleep(1000); // wait to read the display

        moveRobot(moveRobotSpeed, 72.0);

        sleep(pauseTimeBetweenMovements);

        moveRobot(-moveRobotSpeed, -24.0);

        sleep(pauseTimeBetweenMovements);

        turnRobot(turnRobotSpeed, 90);

        sleep(pauseTimeBetweenMovements);

        moveRobot(moveRobotSpeed, 72.0);

        sleep(pauseTimeBetweenMovements);

        turnRobot(turnRobotSpeed, 120);

        sleep(pauseTimeBetweenMovements);

        moveRobot(moveRobotSpeed, 55.4);

        sleep(pauseTimeBetweenMovements);

        turnRobot(turnRobotSpeed, 60);

        sleep(pauseTimeBetweenMovements);

        moveRobot(moveRobotSpeed, 48.0);

        sleep(pauseTimeBetweenMovements);

        turnRobot(turnRobotSpeed, -270);

        sleep(4000);
    }

    /**
     * @author scottarmstrong, harrison chabinksy
     * @version 1.0 - 9/28/16
     * Method that moves the robot in a positive or negative direction, parameters most both be positive or both be negative
     *
     * @param speed is a double of the power that will be given to the robot's motors
     * @param inches is a double of how many inches the robot will travel
     *
     **/
    public void moveRobot (double speed, double inches) {

        robot.resetEncoderData();

        double rotations = inches / (Math.PI * HardwareDriveBot.WHEEL_DIAMETER);
        int encTarget =  (int)rotations * HardwareDriveBot.ENC_ROTATION;

        robot.motorLeft.setPower(speed);
        robot.motorRight.setPower(speed);

        DcMotor motorToTrack = null;

        if (speed > 0 && inches > 0) {

            motorToTrack = robot.motorLeft;

        } else {

            motorToTrack = robot.motorRight;
        }

        waitThenStopRobot(encTarget, motorToTrack);

        System.out.println("Encoder" + motorToTrack.getCurrentPosition());
        Log.i("ROBOT", "Encoder" + motorToTrack.getCurrentPosition());
    }

    /**
     * @author scottarmstrong, harrison chabinsky
     * @version 1.0 - 9/28/16
     *
     * Turns robot a certain amount of degrees on the spot
     *
     * @param speed double of power to be given to the motors
     * @param degrees double of degrees that the robot should rotate
     */
    public void turnRobot (double speed, double degrees) {

        robot.resetEncoderData();

        double numOfRotationsFor360Turn = 3.325;
        double rotations = numOfRotationsFor360Turn * (degrees / 360);

        int encTarget = (int) rotations * HardwareDriveBot.ENC_ROTATION;

        DcMotor motorToTrack = robot.motorRight;

        if (degrees < 0) {

            robot.motorLeft.setPower(speed);
            robot.motorRight.setPower(-speed);

        } else {

            robot.motorLeft.setPower(-speed);
            robot.motorRight.setPower(speed);
        }

        waitThenStopRobot(encTarget, motorToTrack);

        updateEncoderData(motorToTrack.getCurrentPosition());
    }

    //wait until we have reached our target position

    /**
     * @author scottarmstrong, harrison chabinsky
     * @version 1.0 - 9/28/16
     *
     * @param encTarget double of the desired encoder ticks before the robot stops
     * @param motorWithPositionToTrack motor that should be compared with the encTarget
     */
    public void waitThenStopRobot (int encTarget, DcMotor motorWithPositionToTrack) {

        if (encTarget > 0) {

            while (motorWithPositionToTrack.getCurrentPosition() < encTarget) {

                idleAndUpdateData(encTarget, motorWithPositionToTrack);
            }
        } else {

            while (motorWithPositionToTrack.getCurrentPosition() > encTarget) {

                idleAndUpdateData(encTarget, motorWithPositionToTrack);
            }
        }

        robot.stop();
    }

    /**
     * @author scottarmstrong, harrison chabinsky
     * @version 1.0 - 9/28/16
     *
     * Waits and updates encoder data
     *
     * @param encTarget encTarget to print
     * @param motorWithPositionToTrack motor with position to add to encoder data
     */
    public void idleAndUpdateData(double encTarget, DcMotor motorWithPositionToTrack) {

        System.out.println(motorWithPositionToTrack.getCurrentPosition() + " < " + encTarget);

        try {
            idle();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        updateEncoderData(motorWithPositionToTrack.getCurrentPosition());
    }

    /**
     * @author scottarmstrong, harrison chabinsky
     * @version 1.0 - 9/28/16
     *
     * Updates the encoder data with the position
     *
     * @param position int position of an encoder
     */
    public void updateEncoderData (int position) {
        telemetry.addData("Encoder", position);
        telemetry.update();
    }
}
