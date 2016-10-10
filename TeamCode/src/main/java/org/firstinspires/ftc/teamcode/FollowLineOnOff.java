package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * SensorLoop - reads several sensors and displays their status or values on the DS phone.
 *
 * @author Jochen Fischer
 * @version 1.0 - 9/18/2016
 * @version 1.1 - 10/4/2016 - added legacy sensors (light, touch, ultrasound) and gamepad control.
 *
 */
@TeleOp(name="Follow Line On Off", group="ElonDev")
// @Disabled
public class FollowLineOnOff extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    // define the robot hardware:
    HardwareDriveBot robot   = new HardwareDriveBot();   // Use the Drivebot hardware
    double onSpeed = 0.4;
    double offSpeed = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {

        // initialize the hardware
        // including the use of encoders
        robot.init(hardwareMap);

        //sensor "calibration"
        int threshold = (5 + 47) / 2;//26


        telemetry.addData("Status", "Initialized");
        telemetry.addData("Threshold", threshold);
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            int brightness = robot.sensorColor.alpha();

            if (brightness > threshold) {
                //right turn

                robot.motorLeft.setPower(onSpeed);
                robot.motorRight.setPower(offSpeed);
            } else {
                //left turn

                robot.motorLeft.setPower(offSpeed);
                robot.motorRight.setPower(onSpeed);
            }

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}