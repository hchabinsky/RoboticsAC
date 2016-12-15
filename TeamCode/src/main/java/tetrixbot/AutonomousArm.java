
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
@TeleOp(name="Autonomous Arm", group="Robotic Arms")
public class AutonomousArm extends LinearOpMode {

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

                boolean blockingFunctionBool = dropObjectInSmallestTube();

            } else {

                startButtonPressed = false;
            }

            //----------------------------------------
            // Sweeping positions
            //----------------------------------------
            if (gamepad1.a) {

                //beginning sweeping position
                robot.posShoulder = robot.INIT_SHOULDER;
                robot.posElbow = robot.INIT_ELBOW;
                robot.posTilt = robot.INIT_TILT;
                robot.posTwist = robot.INIT_TWIST;
                robot.posSweeper = robot.SWEEPER_GO;
            }

            if (gamepad1.b) {

                //take ball out of harm's way
                robot.posSweeper = robot.SWEEPER_STOP;

                int upShoulder = -405;
                int upElbow = -1066;

                robot.posShoulder = upShoulder;
                robot.posElbow = upElbow;

                robot.posTilt = .388;
            }

            if (gamepad1.x) {

                boolean holdUpLoop = dropObjectInSmallestTube();
            }

            robot.servoContainerTilt.setPosition(robot.posTilt);

            robot.servoSweeper.setPosition(robot.posSweeper);


            //----------------------------------------
            // Drop ball positions
            //----------------------------------------

            if (gamepad1.left_stick_button) {

                boolean whileLoopStopper = dropObjectInSmallestTube();

            }


            //----------------------------------------
            // move WHEELS
            //----------------------------------------
            double speedLeft = gamepad1.left_stick_y;
            double speedRight = gamepad1.right_stick_y;

            speedLeft = Range.clip(speedLeft ,-1,1);
            speedRight = Range.clip(speedRight,-1,1);

            robot.motorLeft.setPower(-speedLeft);
            robot.motorRight.setPower(-speedRight);


            //

            ///
            /*




            Yoooooo makes right stick control right wheel and left stick control left wheel

            For ramp we need material to put on wheels but as far as dropping a ball in the goal,
             find the base of the goal with the light sensor then back up and drop that shit in






            */

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

            telemetry.addData("Shoulder:",robot.posShoulder);

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

            telemetry.addData("Elbow:", robot.posElbow);


            //----------------------------------------
            // Container Tilt
            //----------------------------------------
            if(gamepad1.dpad_down) {
                // tilt down
                robot.posTilt = Range.clip(robot.posTilt + robot.DELTA_TILT,
                        robot.MIN_TILT, robot.MAX_TILT);
            }
            if(gamepad1.dpad_up) {
                // tilt up
                robot.posTilt = Range.clip(robot.posTilt - robot.DELTA_TILT,
                        robot.MIN_TILT, robot.MAX_TILT);
            }
            robot.servoContainerTilt.setPosition(robot.posTilt);
            telemetry.addData("Tilt position:",robot.posTilt);

            //----------------------------------------
            // Container Twist
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
            telemetry.addData("Twist:", robot.posTwist);

            telemetry.update();

            robot.waitForTick(50);
        }
    }

    /**
     * lifts cube to a position over the box and drops it in before returning to init positions
     *
     * @return boolean to temporarily stop loop
     * @throws InterruptedException
     */
    public boolean dropObjectInSmallestTube() throws InterruptedException {

        double hangBalls = .4;
        double dumpBalls = .43;
        int tiltBoxOverParallel = -964;

        robot.servoContainerTilt.setPosition(hangBalls);

        telemetry.addData("Tilt position:",robot.posTilt);
        telemetry.update();

        double currentTime = runtime.milliseconds();

        //wait to position hanging box
        while (currentTime + 3000 > runtime.milliseconds()) {

            robot.servoContainerTilt.setPosition(hangBalls);

            telemetry.addData("first loop tilt position:",robot.posTilt);
            telemetry.update();
            idle();
        }

        setMotorTargetAndPower(robot.motorElbow, tiltBoxOverParallel, robot.POWER_ELBOW);

        while(robot.motorElbow.isBusy()) {
            idle();
        }

        telemetry.addData("Elbow moved up", "yep");

        robot.servoContainerTilt.setPosition(dumpBalls);


        while (currentTime + 2000 > System.currentTimeMillis()) {
            robot.servoContainerTilt.setPosition(dumpBalls);

            telemetry.addData("second loop tilt position:",robot.posTilt);
            telemetry.update();
            idle();
        }


        return true;
    }


    public boolean dropObjectInBiggestTube() throws InterruptedException {
        int upShoulder = -405;
        int upElbow = -1066;
        double boxTilt = .416;

        double currentTime = runtime.milliseconds();

        setMotorTargetAndPower(robot.motorShoulder, upShoulder, robot.POWER_SHOULDER);
        setMotorTargetAndPower(robot.motorElbow, upElbow, robot.POWER_ELBOW);

        // wait for motors to lift cube
        while (robot.motorShoulder.isBusy()) {
            idle();
        }

        // move servos to box
        robot.servoContainerTwist.setPosition(robot.INIT_TWIST);

        int timeDelay = 2000;

        // wait for arm to position itself over box
        while (runtime.milliseconds() < currentTime + timeDelay) {
            idle();
        }

        // drop cube
        robot.servoContainerTilt.setPosition(boxTilt);

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
