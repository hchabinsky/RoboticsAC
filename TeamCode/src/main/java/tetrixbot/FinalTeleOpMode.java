
package tetrixbot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Initializes arm motors using limit switches, allows user to pick up a cube and
 * autonomously takes the cube to a box with a set start position
 *
 * @author scottarmstrong, harrisonchabinsky
 */
@TeleOp(name="Final TeleOp Mode", group="Robotic Arms")
public class FinalTeleOpMode extends LinearOpMode {

    HardwareTetrixBot robot   = new HardwareTetrixBot();

    private ElapsedTime runtime = new ElapsedTime();

    private boolean startButtonPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        initializeRobot();
        telemetry.addData("robot", "initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while(opModeIsActive()) {

            int currentShoulderPosition = robot.motorShoulder.getCurrentPosition();
            int currentElbowPosition = robot.motorElbow.getCurrentPosition();

            if(gamepad1.start && !startButtonPressed) {

                startButtonPressed = true;

                boolean blockingFunctionBool = dropObject();

            } else {

                startButtonPressed = false;
            }

            //----------------------------------------
            // move SHOULDER
            //----------------------------------------

            if(gamepad1.left_trigger > 0.5 && !robot.sensorShoulderLimit.isPressed()) {
                // move shoulder up
                robot.posShoulder = currentShoulderPosition
                        + robot.DELTA_SHOULDER;
            }
            if(gamepad1.left_bumper) {
                // move shoulder down
                robot.posShoulder = currentShoulderPosition
                        - robot.DELTA_SHOULDER;
            }

            robot.motorShoulder.setTargetPosition(robot.posShoulder);
            robot.motorShoulder.setPower(robot.POWER_SHOULDER);

            //----------------------------------------
            // move ELBOW
            //----------------------------------------

            if (gamepad1.right_bumper && !robot.sensorElbowLimit.isPressed()) {
                // move elbow up:
                robot.posElbow = currentElbowPosition
                        + robot.DELTA_ELBOW;
            }

            if (gamepad1.right_trigger > 0.5) {
                // move elbow up:
                robot.posElbow = currentElbowPosition
                        - robot.DELTA_ELBOW;
            }

            robot.motorElbow.setTargetPosition(robot.posElbow);
            robot.motorElbow.setPower(robot.POWER_ELBOW);

            //----------------------------------------
            // move WRIST
            //----------------------------------------

            if(gamepad1.left_stick_y > 0) {
                // move left, CCW
                robot.posTilt = Range.clip(robot.posTilt + robot.DELTA_TILT,
                        robot.MIN_TILT, robot.MAX_TILT);
            }
            if(gamepad1.left_stick_y < 0) {
                // move right, CC
                robot.posTilt = Range.clip(robot.posTilt - robot.DELTA_TILT,
                        robot.MIN_TILT, robot.MAX_TILT);
            }
            robot.servoContainerTilt.setPosition(robot.posTilt);

            //----------------------------------------
            // move HAND
            //----------------------------------------

            if(gamepad1.dpad_right) {
                // move left, CCW
                robot.posTwist = Range.clip(robot.posTwist + robot.DELTA_TWIST,
                        robot.MIN_TWIST, robot.MAX_TWIST);
            }
            if(gamepad1.dpad_left) {
                // move right, CC
                robot.posTwist = Range.clip(robot.posTwist - robot.DELTA_TWIST,
                        robot.MIN_TWIST, robot.MAX_TWIST);
            }
            robot.servoContainerTwist.setPosition(robot.posTwist);

            //----------------------------------------
            // move GRIPPER
            //----------------------------------------

            if(gamepad1.right_stick_y > 0) {
                robot.posSweeper = robot.posSweeper + robot.DELTA_SWEEPER;
            }
            if (gamepad1.right_stick_y < 0) {
                robot.posSweeper = robot.posSweeper - robot.DELTA_SWEEPER;
            }

            robot.servoSweeper.setPosition(robot.posSweeper);

            robot.waitForTick(50);
        }
    }

    /**
     * lifts cube to a position over the box and drops it in before returning to init positions
     *
     * @return boolean to temporarily stop loop
     * @throws InterruptedException
     */
    public boolean dropObject() throws InterruptedException {
        int upShoulder = -1027;
        int upElbow = -467;
        double torsoToBox = .38;

        double currentTime = runtime.milliseconds();

        setMotorTargetAndPower(robot.motorShoulder, upShoulder, robot.POWER_SHOULDER);
        setMotorTargetAndPower(robot.motorElbow, upElbow, robot.POWER_ELBOW);

        // wait for motors to lift cube
        while (robot.motorShoulder.isBusy()) {
            idle();
        }

        // move servos to box
        robot.servoContainerTilt.setPosition(robot.MAX_TILT);
        robot.servoContainerTwist.setPosition(robot.INIT_TWIST);

        int timeDelay = 5000;

        // wait for arm to position itself over box
        while (runtime.milliseconds() < currentTime + timeDelay) {
            idle();
        }

        // drop cube
//        robot.servoSweeper.setPosition(robot.MAX_SWEEPER);

        currentTime = runtime.milliseconds();

        while (runtime.milliseconds() < currentTime + timeDelay) {
            idle();
        }

        // set arm to beginning position for returning control to loop
        robot.posShoulder = robot.INIT_SHOULDER;
        robot.posElbow = robot.INIT_ELBOW;

        return true;
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

        double currentTime = runtime.milliseconds();

        initServos();

        int timeDelay = 2000;

        // wait for servos to initialize
        while (runtime.milliseconds() < currentTime + timeDelay) {
            idle();
        }

        setMotorsToStartingPos();

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
            telemetry.addData(motor.getDeviceName() + "'s Position: ", motor.getCurrentPosition());
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
