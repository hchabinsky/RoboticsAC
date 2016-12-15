
package tetrixbot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Controls the robot arm's motors and servos with a gaming controller
 *
 * @author scottarmstrong, harrisonchabinsky
 */
@TeleOp(name="Move Arm", group="Robotic Arms")
public class MoveArm extends LinearOpMode {

    HardwareTetrixBot robot   = new HardwareTetrixBot();

    private ElapsedTime runtime = new ElapsedTime();

    boolean startButtonPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            //------------------------------------------------------------------
            // reset shoulder and elbow encoders by pressing the Start button
            //------------------------------------------------------------------
            if (gamepad1.start && !startButtonPressed) {

                // start button state went from not pressed to pressed:
                startButtonPressed = true;
                robot.resetMotorEncoder(robot.motorShoulder);
                robot.resetMotorEncoder(robot.motorElbow);
                robot.posElbow = 0;
                robot.posShoulder = 0;
                robot.robotIsInitialized = true;
            }
            else {
                startButtonPressed = false;
            }

            //----------------------------------------
            // move shoulder up and down
            //----------------------------------------
            int currentShoulderPosition = robot.motorShoulder.getCurrentPosition();

            if(gamepad1.left_trigger > 0.5) {
                // move shoulder down:
                robot.posShoulder = currentShoulderPosition
                        + robot.DELTA_SHOULDER;
            }
            if(gamepad1.left_bumper) {
                // move shoulder up:
                robot.posShoulder = currentShoulderPosition
                        - robot.DELTA_SHOULDER;
            }

            telemetry.addData("Shoulder Position: ", robot.posShoulder);

            robot.motorShoulder.setTargetPosition(robot.posShoulder);
            robot.motorShoulder.setPower(robot.POWER_SHOULDER);

            // move elbow up and down
            int currentElbowPosition = robot.motorElbow.getCurrentPosition();

            if (gamepad1.right_bumper) {
                // move elbow up:
                robot.posElbow = currentElbowPosition
                        + robot.DELTA_ELBOW;
            }

            if (gamepad1.right_trigger > 0.5) {
                // move elbow up:
                robot.posElbow = currentElbowPosition
                        - robot.DELTA_ELBOW;
            }

            telemetry.addData("Elbow Position: ", robot.posElbow);

            robot.motorElbow.setTargetPosition(robot.posElbow);
            robot.motorElbow.setPower(robot.POWER_ELBOW);


            // Move box up and down

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

            telemetry.addData("Box Vertical Position: ", robot.posTilt);

            robot.servoContainerTilt.setPosition(robot.posTilt);

            // move hand

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

            telemetry.addData("Box Twist Position: ", robot.posTwist);

            robot.servoContainerTwist.setPosition(robot.posTwist);

            // move gripper

            if(gamepad1.right_stick_y > 0) {
                robot.posSweeper = robot.posSweeper + robot.DELTA_SWEEPER;
            }
            if (gamepad1.right_stick_y < 0) {
                robot.posSweeper = robot.posSweeper - robot.DELTA_SWEEPER;
            }

            telemetry.addData("Sweeper Position: ", robot.posSweeper);

            robot.servoSweeper.setPosition(robot.posSweeper);

            telemetry.update();

            // run the loop in 50ms increments (and give up the CPU for the rest of a cycle)
            robot.waitForTick(50);
        }
    }
}
