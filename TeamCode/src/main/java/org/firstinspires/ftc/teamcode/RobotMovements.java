package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Template: Linear OpMode", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class RobotMovements extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareDriveBot robot = new HardwareDriveBot();

    private ElapsedTime runtime = new ElapsedTime();
    // DcMotor leftMotor = null;
    // DcMotor rightMotor = null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            idle();
        }
    }

    /**
     * @author Jochen Fischer
     * @version 1.0 - 9/25/2016 - inital version, works only going forward
     * @version 1.1 - 9/26/2016 - added code to go in both directions
     * @version 2.0 - 10/4/2016 - extensive use of functions
     *
     * moveRobot - Moves the robot by a given distance and speed.
     *
     * @param speed    robot speed between -1.0 ... 1.0
     * @param inches   driving distance in inces
     * @throws InterruptedException
     */
    private void moveRobot(double speed, double inches) throws InterruptedException {

        // determine if the robot moves forward or backward:
        double direction = Math.signum(speed * inches);

        // sanity check: don't do anything if either speed or inches is zero
        if (direction == 0.0) return;

        // since we know in which direction the robot moves,
        // we can use absolute (=positive) values for both the encoder value and target

        // translate the distance in inches to encoder ticks:
        int encoderTarget = HardwareDriveBot.convertInchesToTicks( Math.abs(inches) );

        // move the desired distance:
        robot.resetEncoderData();
        robot.setRobotSpeed(Math.abs(speed) * direction);

        while (Math.abs(robot.motorLeft.getCurrentPosition()) < encoderTarget) {

            idle();
        }
        stop();
    }

}
