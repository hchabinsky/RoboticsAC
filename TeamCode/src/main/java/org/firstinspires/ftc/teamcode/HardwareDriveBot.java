package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Defines variables for parts of the robots hardware and basic methods that calculate values from
 * the hardware variables as well as move the robot
 *
 * @author scottarmstrong, harrisonchabinsky
 */
public class HardwareDriveBot {

    /* Public OpMode members. */

    // hardware specific constants:
    public static final double SLOW_POWER = 0.2;
    public static final double POWER = 1.0;
    public static final double STOP = 0.0;
    public static final int ENC_ROTATION = 1120;
    public static final double WHEEL_DIAMETER = 4.06;
    public static final double WHEEL_BASE = 10.5;//need to actually measure this


    //~~~~~~~~~~ motors ~~~~~~~~~~
    public DcMotor  motorLeft   = null;
    public DcMotor  motorRight  = null;

    //~~~~~~~~~~ sensors ~~~~~~~~~~
    public TouchSensor sensorTouch = null;
    public ColorSensor sensorColor = null;
    public GyroSensor sensorGyro = null;
    public TouchSensor sensorLegoTouch = null;
    public LightSensor sensorLegoLight = null;
    public UltrasonicSensor sensorUltrasonic = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareDriveBot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        motorRight   = hwMap.dcMotor.get("motorRight");
        motorLeft  = hwMap.dcMotor.get("motorLeft");
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        motorLeft.setPower(STOP);
        motorRight.setPower(STOP);

        // reset encoders
        resetEncoderData();


        //Set up the sensors

        sensorTouch = hwMap.touchSensor.get("sensorTouch");
        sensorColor = hwMap.colorSensor.get("sensorColor");
        //sensorGyro = hwMap.gyroSensor.get("sensorGyro");
        sensorLegoLight = hwMap.lightSensor.get("sensorLegoLight");
        sensorLegoTouch = hwMap.touchSensor.get("sensorLegoTouch");
        sensorUltrasonic = hwMap.ultrasonicSensor.get("sensorUltrasonic");

        sensorColor.enableLed(true);
        sensorLegoLight.enableLed(true);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    public void stop() {

        motorLeft.setPower(STOP);
        motorRight.setPower(STOP);
    }

    /**
     * Makes the robot turn in a counter clockwise direction
     *
     * @param speed the robot should turn at
     */
    public void spin(double speed) {

        motorLeft.setPower(-speed);
        motorRight.setPower(speed);
    }

    public void setRobotSpeed(double speed) {

        motorLeft.setPower(speed);
        motorRight.setPower(speed);
    }


    /**
     *  Coverts encoder ticks to inches
     *
     * @param encTicks number of encTicks to be converted
     * @return int of the corresponding number of inches
     */
    public double convertTicksToInches(int encTicks) {

        return Math.PI * WHEEL_DIAMETER * (encTicks / ENC_ROTATION);
    }

    /**
     *  Coverts inches into encoder ticks
     *
     * @param inches number of inches to be converted
     * @return int of the corresponding number of encoder ticks
     */
    public static int convertInchesToTicks(double inches) {

        // translate the distance in inches to encoder ticks:
        double wheelRotations = inches / (Math.PI * HardwareDriveBot.WHEEL_DIAMETER);
        int encoderTicks = (int)(wheelRotations * HardwareDriveBot.ENC_ROTATION);

        return encoderTicks;
    }

    /**
     * convertDegreesToTicks - convert turn angle to encoder ticks
     *
     * @param degrees  turn angle of the robot, positive values are clockwise
     * @return int of the corresponding number of encoder ticks
     */
    public static int convertDegreesToTicks(double degrees) {

        double wheelRotations = (degrees / 360.0) * Math.PI * HardwareDriveBot.WHEEL_BASE
                / (Math.PI * HardwareDriveBot.WHEEL_DIAMETER);
        int encoderTarget = (int)(wheelRotations * HardwareDriveBot.ENC_ROTATION);

        return encoderTarget;
    }

    public void resetEncoderData () {

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

