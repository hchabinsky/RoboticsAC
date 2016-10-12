package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Counts the pieces of tape that the robot moves over before the robot's touchSensor is pressed
 *
 * @author scottarmstrong, harrisonchabinsky
 */
@TeleOp(name="CountTape", group="Elon")
public class CountTape extends LinearOpMode {

    // define the robot hardware
    HardwareDriveBot robot = new HardwareDriveBot();

    @Override
    public void runOpMode() throws InterruptedException {

        // initialize the robot
        robot.init(hardwareMap);

        robot.resetEncoderData();

        waitForStart();

        robot.setRobotSpeed(HardwareDriveBot.SLOW_POWER);

        int encTicksToBox = 0;
        int numOfWhiteLines = 0;
        int inchesToGoBeforeCounting = 12;
        int wiggleRoomForSensors = 5;
        int encTicksToWaitBeforeCountingLines = robot.convertInchesToTicks(inchesToGoBeforeCounting);

        ColorData lowThresholds = new ColorData(Integer.MAX_VALUE, Integer.MAX_VALUE, Integer.MAX_VALUE, Integer.MAX_VALUE);
        ColorData highThresholds = new ColorData(0, 0, 0, 0);

        boolean overTape = false;
        boolean doCountLines = false;

        while (robot.sensorTouch.isPressed() == false) {

            if (doCountLines) {

                Boolean[] robotPositionInfo = findRobotPosition(highThresholds, lowThresholds, new ColorData(robot.sensorColor.red(), robot.sensorColor.green(),
                        robot.sensorColor.blue(), robot.sensorColor.alpha()), overTape, wiggleRoomForSensors);

                overTape = robotPositionInfo[0];

                boolean addToLineTotal = robotPositionInfo[1];

                if (addToLineTotal) {

                    numOfWhiteLines += 1;
                }

                //Using telemetry calls to display the number of lines as they are found
                telemetry.addData("Number of lines", numOfWhiteLines);

            } else {

                ColorData currentColorData = new ColorData(robot.sensorColor.red(), robot.sensorColor.green(),
                        robot.sensorColor.blue(), robot.sensorColor.alpha());

                highThresholds = updateMax(currentColorData, highThresholds);

                lowThresholds = updateMin(currentColorData, lowThresholds);

                doCountLines = robot.motorLeft.getCurrentPosition() > encTicksToWaitBeforeCountingLines;
            }

            encTicksToBox = robot.motorLeft.getCurrentPosition();

            //Using telemetry calls to display the traveled distance in inches as the robot moves toward the wall
            telemetry.addData("Inches traveled before box:", robot.convertTicksToInches(encTicksToBox));
            telemetry.update();

            idle();
        }

        robot.stop();


        double inches = robot.convertTicksToInches(encTicksToBox);

        //Showing the traveled distance in inches on the Android Studio logcat once the wall was hit
        Log.i("Total Inches To Box", Double.toString(inches));

        robot.setRobotSpeed(-HardwareDriveBot.SLOW_POWER);

        while (robot.motorLeft.getCurrentPosition() > 0) {

            idle();
        }

        robot.stop();

        //Showing the thresholds from the calibration on the Android Studio console (requirement)
        System.out.println("Low thresholds: " + new ColorData(lowThresholds.getRed(), lowThresholds.getBlue(), lowThresholds.getGreen(), lowThresholds.getAlpha()));
        System.out.println("High thresholds: " + new ColorData(highThresholds.getRed(), highThresholds.getBlue(), highThresholds.getGreen(), highThresholds.getAlpha()));

        sleep(4000);
    }

    public ColorData updateMax (ColorData currentColorData, ColorData highThresholds) {

        if (currentColorData.getRed() > highThresholds.getRed()) {

            highThresholds.setRed(currentColorData.getRed());
        }
        if (currentColorData.getGreen() > highThresholds.getGreen()) {

            highThresholds.setGreen(currentColorData.getGreen());
        }
        if (currentColorData.getBlue() > highThresholds.getBlue()) {

            highThresholds.setBlue(currentColorData.getBlue());
        }
        if (currentColorData.getAlpha() > highThresholds.getAlpha()) {

            highThresholds.setAlpha(currentColorData.getAlpha());
        }

        return highThresholds;
    }

    public ColorData updateMin (ColorData currentColorData, ColorData lowThresholds) {

        if (currentColorData.getRed() < lowThresholds.getRed()) {

            lowThresholds.setRed(currentColorData.getRed());
        }
        if (currentColorData.getGreen() < lowThresholds.getGreen()) {

            lowThresholds.setGreen(currentColorData.getGreen());
        }
        if (currentColorData.getBlue() < lowThresholds.getBlue()) {

            lowThresholds.setBlue(currentColorData.getBlue());
        }
        if (currentColorData.getAlpha() < lowThresholds.getAlpha()) {

            lowThresholds.setAlpha(currentColorData.getAlpha());
        }

        return lowThresholds;
    }

    public Boolean[] findRobotPosition(ColorData highThresholds, ColorData lowThresholds, ColorData currentColorData, boolean overTape, int wiggleRoomForSensors) {

        Boolean[] robotPositionInfo = new Boolean[2];
        boolean addToNumOfWhiteLines = false;

        if (currentColorData.allColorsAboveThreshold(highThresholds.getRed() - wiggleRoomForSensors, highThresholds.getBlue() - wiggleRoomForSensors,
                highThresholds.getGreen() - wiggleRoomForSensors, highThresholds.getAlpha() - wiggleRoomForSensors) && overTape == false) {

            overTape = true;
            addToNumOfWhiteLines = true;
        }

        if (currentColorData.allColorsBelowThreshold(lowThresholds.getRed() + wiggleRoomForSensors, lowThresholds.getBlue() + wiggleRoomForSensors,
                lowThresholds.getGreen() + wiggleRoomForSensors, lowThresholds.getAlpha() + wiggleRoomForSensors)) {

            overTape = false;
        }

        robotPositionInfo[0] = overTape;
        robotPositionInfo[1] = addToNumOfWhiteLines;

        return robotPositionInfo;
    }
}
