
package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * FollowLineP - Line follower using a basic proportional controller
 *
 * @author Jochen Fischer
 * @version 1.0 - 10/10/2016 as shown in class
 *
 *
 */
@Autonomous(name="Follow Line P", group="ElonDev")
// @Disabled
public class FollowLineP extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    // define the robot hardware:
    HardwareDriveBot robot   = new HardwareDriveBot();   // Use the Drivebot hardware

    // parameters used by the controller:
    private double speed = 0.2;
    private int reference = (5+47)/2;
    private double Kc = 0.01;
    private double Kp = 0.45 * Kc;
    private double Ki = 0.54 * Kc;
    private double Kd = 0.0;
    private double Pc = 0.339285714;


    private double dt = 50.0;  // interval in millisconds
    private double dT = dt/1000.0;   // interval in secondes

    private int loopCounter = 0;

    @Override
    public void runOpMode() throws InterruptedException {


        /**NOT THE P CONTROLLER TO BE GRADED, THAT'S PFollowLineController**/


        // initialize the hardware
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // print process parameters in console:
        Log.i("ROBOT", String.format("ref = %d",reference));

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();




        // run until the end of the match (driver presses STOP)
        while (runtime.seconds() < 10) {

            // read the current light sensor value:
            int brightness = robot.sensorColor.alpha();

            // implementation of P-controller:
            double error = reference - brightness;

            double turn = Kc * error;

            robot.motorLeft.setPower(speed - turn);
            robot.motorRight.setPower(speed + turn);

            String message = String.format("%s %d",
                    runtime.toString(), brightness);
            Log.i("ROBOT", message);

            // wait for the next time slot:
            loopCounter++;
            double nextTimeSlot = loopCounter * dt;

            while (runtime.milliseconds() < nextTimeSlot) {
                idle();
            }
        }
        robot.stop();
    }
}
