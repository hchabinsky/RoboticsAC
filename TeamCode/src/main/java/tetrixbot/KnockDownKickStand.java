package tetrixbot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;


/**
 * MoveWithEncoder moves robot through an obsticle course
 *
 * @author scottarmstrong, harrisonchabinsky
 */

@Autonomous(name="Knock Down Kickstand", group="Elon")
public class KnockDownKickStand extends LinearOpMode {

    // define the robot hardware
    HardwareTetrixBot robot = new HardwareTetrixBot();
    double moveRobotToRampSpeed = 0.1;
    double moveRobotToAttackRampSpeed = 0.6;
    double turnRobotSpeed = 0.2;

    @Override
    public void runOpMode() throws InterruptedException {

        // initialize the robot
        robot.init(hardwareMap);

        initializeRobot();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)

        waitForStart();

        //shoulder -433
        //elbow -958
        //tilt .388
        // twist .441

        updateEncoderData(robot.motorLeft.getCurrentPosition());

        int pauseTimeBetweenMovements = 250;

        sleep(1000); // wait to read the display

        moveRobot(moveRobotToRampSpeed, -46.0);

        sleep(pauseTimeBetweenMovements);


        turnRobot(turnRobotSpeed, 95 * 2);

        moveRobot(moveRobotToAttackRampSpeed, -26);

//        moveRobot(moveRobotToRampSpeed, 8);
//
//        turnRobot(turnRobotSpeed, -60 * 2);
//
//        moveRobot(moveRobotToAttackRampSpeed, -26);

    }

    private void moveRobot(double speed, double inches) throws InterruptedException {

        // determine if the robot moves forward or backward:
        double direction = Math.signum(speed * inches);

        // sanity check: don't do anything if either speed or inches is zero
        if(direction == 0.0) return;

        // since we know in which direction the robot moves,
        // we can use absolute (=positive) values for both the encoder value and target

        // translate the distance in inches to encoder ticks:
        int encoderTarget = HardwareTetrixBot.convertInchesToTicks( Math.abs(inches) );

        // move the desired distance:
        robot.resetEncoderData();
        robot.setRobotSpeed(Math.abs(speed) * direction);

        while (Math.abs(robot.motorLeft.getCurrentPosition()) < encoderTarget) {
            idle();
        }
        robot.stop();
    }

    private void turnRobot(double speed, double degrees) throws InterruptedException {

        // determine if the robot moves left (1.0) or right (-1.0):
        double direction = Math.signum(speed * degrees);

        // sanity check: don't do anything if either speed or inches is zero
        if(direction == 0.0) return;

        // since we know in which direction the robot moves,
        // we can use absolute (=positive) values for both the encoder value and target

        // translate the degrees into encoder ticks:
        int encoderTarget = HardwareTetrixBot.convertDegreesToTicks( Math.abs(degrees) );

        // move the desired distance:
        robot.resetEncoderData();
        robot.spin(Math.abs(speed) * direction);
        while (Math.abs(robot.motorLeft.getCurrentPosition()) < encoderTarget) {
            idle();
        }
        robot.stop();
    }

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

                System.out.println("entered this while loop");

                updateEncoderData(motorWithPositionToTrack.getCurrentPosition());

                idleAndUpdateData(encTarget, motorWithPositionToTrack);
            }
        } else {

            while (motorWithPositionToTrack.getCurrentPosition() > encTarget) {

                System.out.println("no bruh this while loop");


                updateEncoderData(motorWithPositionToTrack.getCurrentPosition());

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

    /**
     * initializes the motors by bringing them to the limit switches, then resetting their encoders.
     * sets all servos and motors to starting positions
     *
     * @throws InterruptedException
     */
    public void initializeRobot() throws InterruptedException {
        // move motors up until limit switch is engaged and reset encoders
        initMotorEncoder(robot.motorShoulder, robot.sensorShoulderLimit, robot.POWER_SHOULDER / 2);

        initMotorEncoder(robot.motorElbow, robot.sensorElbowLimit, robot.POWER_ELBOW / 2);

        double currentTime = System.currentTimeMillis();

        initServos();

        int timeDelay = 2000;

        // wait for servos to initialize
        while (System.currentTimeMillis() < currentTime + timeDelay) {
            idle();
        }

        robot.robotIsInitialized = true;
    }

    public void initServos() {

        robot.servoContainerTilt.setPosition(robot.posTilt);
        robot.servoContainerTwist.setPosition(robot.posTwist);
        robot.servoSweeper.setPosition(robot.posSweeper);
    }

    /**
     * moves motor up till it hits limit switch, then resets encoders
     *
     * @param motor motor to initialize
     * @param limitSensor touch sensor to hit
     * @param power power to move to the button with
     * @throws InterruptedException
     */
    public void initMotorEncoder(DcMotor motor, TouchSensor limitSensor, double power) throws InterruptedException {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(power);

        while (!limitSensor.isPressed()) {
            idle();
        }
        motor.setPower(robot.STOP);
        robot.resetMotorEncoder(motor);
    }

    public void setMotorTargetAndPower(DcMotor motor, int position, double power) {
        motor.setTargetPosition(position);
        motor.setPower(power);
    }

    public void setMotorsToStartingPos() {
        setMotorTargetAndPower(robot.motorShoulder, robot.INIT_SHOULDER, robot.POWER_SHOULDER);
        setMotorTargetAndPower(robot.motorElbow, robot.INIT_ELBOW, robot.POWER_ELBOW);
    }
}
