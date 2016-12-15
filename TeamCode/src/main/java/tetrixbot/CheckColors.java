
package tetrixbot;

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
@TeleOp(name="SensorLoop", group="ElonDev")
// @Disabled
public class CheckColors extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    // define the robot hardware:
    HardwareTetrixBot robot   = new HardwareTetrixBot();   // Use the Drivebot hardware

    @Override
    public void runOpMode() throws InterruptedException {

        // initialize the hardware
        // including the use of encoders
        robot.init(hardwareMap);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // turn active lights on color and light sensor on and off:
            robot.sensorColor.enableLed(true);

            // read the color sensor:
            int red = robot.sensorColor.red();
            int green = robot.sensorColor.green();
            int blue = robot.sensorColor.blue();
            int alpha = robot.sensorColor.alpha();



            // add telemetry data:
            telemetry.addData("Color", String.format("red=%3d G=%3d B=%3d A=%3d", red, green, blue, alpha));

            telemetry.update();

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
