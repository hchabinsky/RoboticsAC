package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * SensorLoop - reads several sensors and displays their status or values on the DS phone.
 *
 * @author Jochen Fischer
 * @version 1.0 - 9/18/2016
 * @version 1.1 - 10/4/2016 - added legacy sensors (light, touch, ultrasound) and gamepad control.
 *
 */
@Autonomous(name="Follow Line P", group="ElonDev")
public class FollowLineP extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    // define the robot hardware:
    HardwareDriveBot robot   = new HardwareDriveBot();   // Use the Drivebot hardware
    double speed = 0.2;
    int reference = (5 + 47) / 2;
    double Kp = 0.005;

    @Override
    public void runOpMode() throws InterruptedException {

        // initialize the hardware
        // including the use of encoders
        robot.init(hardwareMap);


        telemetry.addData("Status", "Initialized");
        telemetry.addData("Reference", reference);
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            int brightness = robot.sensorColor.alpha();

            double error = reference - brightness;

            double turn = error * Kp;

            robot.motorLeft.setPower(speed - turn);
            robot.motorRight.setPower(speed + turn);

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}