package tetrixbot;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Defines variables for parts of the robots hardware and basic methods that calculate values from
 * the hardware variables as well as move the robot
 *
 * @author scottarmstrong, harrisonchabinsky
 */
public class HardwareTetrixBot {

    /* Public OpMode members. */

    //--------------------------------------
    // motors
    //--------------------------------------
    public DcMotor  motorShoulder = null;

    final static int DELTA_SHOULDER = 40;
    final static int INIT_SHOULDER  = -1500;
    int posShoulder = INIT_SHOULDER;
    final static double POWER_SHOULDER = 0.2;

    public DcMotor motorElbow = null;

    final static int DELTA_ELBOW = 40;
    final static int INIT_ELBOW  = -1455;
    int posElbow = INIT_ELBOW;
    final static double POWER_ELBOW = 0.2;

    public DcMotor motorRight = null;

    public DcMotor motorLeft = null;

    // hardware specific constants:
    public static final int ENC_ROTATION_40 = 1120;
    public static final int ENC_ROTATION_60 = 1680;     // 1120 * 60 / 40

    // useful constants:
    // hardware specific constants:
    public static final double SLOW_POWER = 0.2;
    public static final double POWER = 1.0;
    public static final double STOP = 0.0;
    public static final int ENC_ROTATION = 1120;
    public static final double WHEEL_DIAMETER = 4.06;
    public static final double WHEEL_BASE = 12.3;

    //--------------------------------------
    // servos
    //--------------------------------------

    Servo servoContainerTilt;

    final static double DELTA_TILT = 0.004;
    final static double MIN_TILT = 0.37;
    final static double MAX_TILT = 1.5;
    final static double INIT_TILT = .388;
    double posTilt = INIT_TILT;

    Servo servoContainerTwist;

    final static double DELTA_TWIST = 0.009;
    final static double MIN_TWIST = .388;
    final static double MAX_TWIST = 1.0;
    final static double INIT_TWIST = .441;
    double posTwist = INIT_TWIST;

    Servo servoSweeper;

    final static double DELTA_SWEEPER = 0.007;
    final static double SWEEPER_STOP = .5;
    final static double SWEEPER_GO = .25;
    double posSweeper = SWEEPER_STOP;

    TouchSensor sensorShoulderLimit;
    TouchSensor sensorElbowLimit;
    ColorSensor sensorColor;


    //--------------------------------------
    // local OpMode members
    //--------------------------------------
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    boolean robotIsInitialized = false;

    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        //--------------------------------------
        // Define and Initialize Arm Motors
        //--------------------------------------
        motorShoulder = hwMap.dcMotor.get("motorShoulder");
        motorShoulder.setDirection(DcMotor.Direction.REVERSE);
        resetMotorEncoder(motorShoulder);

        motorElbow = hwMap.dcMotor.get("motorElbow");
        motorElbow.setDirection(DcMotor.Direction.REVERSE);
        resetMotorEncoder(motorElbow);

        //--------------------------------------
        // Define and Initialize Driving Motors
        //--------------------------------------
        motorRight   = hwMap.dcMotor.get("motorRight");
        motorLeft  = hwMap.dcMotor.get("motorLeft");
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        motorLeft.setPower(STOP);
        motorRight.setPower(STOP);

        resetEncoderData();

        //--------------------------------------
        // Define and initialize servos:
        //--------------------------------------

        servoContainerTilt = hwMap.servo.get("servoContainerTilt");

        servoContainerTwist = hwMap.servo.get("servoContainerTwist");

        servoSweeper = hwMap.servo.get("servoSweeper");

        sensorShoulderLimit = hwMap.touchSensor.get("sensorShoulderLimit");
        sensorElbowLimit = hwMap.touchSensor.get("sensorElbowLimit");
        sensorColor = hwMap.colorSensor.get("sensorColor");
    }

    /**
     *  Coverts inches into encoder ticks
     *
     * @param inches number of inches to be converted
     * @return int of the corresponding number of encoder ticks
     */
    public static int convertInchesToTicks(double inches) {

        // translate the distance in inches to encoder ticks:
        double wheelRotations = inches / (Math.PI * WHEEL_DIAMETER);
        int encoderTicks = (int)(wheelRotations * ENC_ROTATION);

        return encoderTicks;
    }


    /**
     * convertDegreesToTicks - convert turn angle to encoder ticks
     *
     * @param degrees  turn angle of the robot, positive values are clockwise
     * @return int of the corresponding number of encoder ticks
     */
    public static int convertDegreesToTicks(double degrees) {

        double wheelRotations = (degrees / 360.0) * Math.PI * WHEEL_BASE
                / (Math.PI * WHEEL_DIAMETER);
        int encoderTarget = (int)(wheelRotations * ENC_ROTATION);

        return encoderTarget;
    }

    /**
     * resetMotorEncoder - stops and resets the a DcMotor encoder
     *
     * @author Scott Armstrong
     * @version 1.0 - 11/6/2016
     */
    public void resetMotorEncoder(DcMotor aMotor) {
        aMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        aMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void resetEncoderData () {

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
}
