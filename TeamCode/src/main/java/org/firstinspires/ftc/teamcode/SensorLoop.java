
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
public class SensorLoop extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    // define the robot hardware:
    HardwareDriveBot robot   = new HardwareDriveBot();   // Use the Drivebot hardware

    @Override
    public void runOpMode() throws InterruptedException {

        // initialize the hardware
        // including the use of encoders
        robot.init(hardwareMap);

        // calibrate the gyroscope:
        telemetry.addData("Status", "calibrating gyroscope");
        telemetry.update();
        robot.sensorGyro.calibrate();

        // make sure the gyro is calibrated:
        while (robot.sensorGyro.isCalibrating())  {
            idle();
        }

        // get the rotation angle from the gyroscope:
        int heading = robot.sensorGyro.getHeading();
        telemetry.addData("Heading", String.format("%03d", heading));

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // move the robot using the gamepad:
            double speed = -gamepad1.left_stick_y  * robot.SLOW_POWER;
            double turn  =  gamepad1.right_stick_x * robot.SLOW_POWER;

            double powerL = Range.clip(speed + turn, -1.0, 1.0 );
            double powerR = Range.clip(speed - turn, -1.0, 1.0 );

            robot.motorLeft.setPower(powerL);
            robot.motorRight.setPower(powerR);

            // turn active lights on color and light sensor on and off:
            if (gamepad1.b) robot.sensorLegoLight.enableLed(true);
            if (gamepad1.x) robot.sensorLegoLight.enableLed(false);
            if (gamepad1.y) robot.sensorColor.enableLed(true);
            if (gamepad1.a) robot.sensorColor.enableLed(false);

            // read the color sensor:
            int red = robot.sensorColor.red();
            int green = robot.sensorColor.green();
            int blue = robot.sensorColor.blue();
            int alpha = robot.sensorColor.alpha();

            // read the Lego light sensor:
            double light = robot.sensorLegoLight.getLightDetected();
            double rawLight = robot.sensorLegoLight.getRawLightDetected();

            // read the touch sensor:
            boolean isPressed = robot.sensorTouch.isPressed();
            String strButton = isPressed ? "PRESSED" : "released";

            // second touch sensor:
            isPressed = robot.sensorLegoTouch.isPressed();
            String strLegoTouch = isPressed ? "PRESSED" : "released";

            // get the rotation angle from the gyroscope:
            heading = robot.sensorGyro.getHeading();

            // ultrasonic sensor:
            double distance = robot.sensorUltrasonic.getUltrasonicLevel();

            // add telemetry data:
            telemetry.addData("Color", String.format("red=%3d G=%3d B=%3d A=%3d", red, green, blue, alpha));
            telemetry.addData("light", String.format("light=%.3f  rawLight=%.3f", light, rawLight));
            telemetry.addData("Touch", "button is " + strButton);
            telemetry.addData("Lego Touch Sensor", "button is " + strLegoTouch);
            telemetry.addData("Ultrasonic", String.format("distance = %.1fcm", distance));
            telemetry.addData("Heading", String.format("%03d", heading));
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
