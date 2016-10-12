package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Move2Sec - moves the robot for 2 seconds
 *
 * @author scottarmstrong, harrisonchabinsky
 */

@Autonomous(name="Move 2 Sec", group="Elon")
public class Move2Sec extends LinearOpMode {

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


        robot.motorLeft.setPower(HardwareDriveBot.SLOW_POWER);
        robot.motorRight.setPower(HardwareDriveBot.SLOW_POWER);
        telemetry.addData("Status", "Running");
        telemetry.update();

        // wait for two seconds
        sleep(2000);

        robot.motorLeft.setPower(HardwareDriveBot.STOP);
        robot.motorRight.setPower(HardwareDriveBot.STOP);
        telemetry.addData("Status", "Stopped");
        telemetry.update();

        sleep(2000);
    }
}
