/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * SensorLoop
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

        ColorData lowThresholds = new ColorData(Integer.MAX_VALUE, Integer.MAX_VALUE, Integer.MAX_VALUE, Integer.MAX_VALUE);

//        int lowRedThreshold = Integer.MAX_VALUE;
//        int lowGreenThreshold = Integer.MAX_VALUE;
//        int lowBlueThreshold = Integer.MAX_VALUE;
//        int lowAlphaThreshold = Integer.MAX_VALUE;

        ColorData highThresholds = new ColorData(0, 0, 0, 0);

//        int highRedThreshold = 0;
//        int highGreenThreshold = 0;
//        int highBlueThreshold = 0;
//        int highAlphaThreshold = 0;

        int wiggleRoomForSensors = 5;

        boolean overTape = false;
        boolean doCountLines = false;

        int encTicksToWaitBeforeCountingLines = robot.convertInchesToTicks(inchesToGoBeforeCounting);

        while (robot.sensorTouch.isPressed() == false) {

            if (doCountLines) {

                Boolean[] robotPositionInfo = findRobotPosition(highThresholds, lowThresholds, new ColorData(robot.sensorColor.red(), robot.sensorColor.green(), robot.sensorColor.blue(), robot.sensorColor.alpha()), overTape, wiggleRoomForSensors);

                overTape = robotPositionInfo[0];

                boolean addToLineTotal = robotPositionInfo[1];

                if (addToLineTotal) {

                    numOfWhiteLines += 1;
                }

                telemetry.addData("Over Tape", overTape);
                telemetry.addData("Number of lines", numOfWhiteLines);

            } else {

                int red = robot.sensorColor.red();
                int green = robot.sensorColor.green();
                int blue = robot.sensorColor.blue();
                int alpha = robot.sensorColor.alpha();

                ColorData currentColorData = new ColorData(red, blue, green, alpha);

                highThresholds = updateMax(currentColorData, highThresholds);

                lowThresholds = updateMin(currentColorData, lowThresholds);

//                if (red < lowRedThreshold) {
//
//                    lowRedThreshold = red;
//                }
//
//                if (red > highRedThreshold) {
//
//                    highRedThreshold = red;
//                }
//
//                if (green < lowGreenThreshold) {
//
//                    lowGreenThreshold = green;
//                }
//
//                if (green > highGreenThreshold) {
//
//                    highGreenThreshold = green;
//                }
//                if (blue < lowBlueThreshold) {
//
//                    lowBlueThreshold = blue;
//                }
//
//                if (blue > highBlueThreshold) {
//
//                    highBlueThreshold = blue;
//                }
//                if (alpha < lowAlphaThreshold) {
//
//                    lowAlphaThreshold = alpha;
//                }
//
//                if (alpha > highAlphaThreshold) {
//
//                    highAlphaThreshold = alpha;
//                }

                doCountLines = robot.motorLeft.getCurrentPosition() > encTicksToWaitBeforeCountingLines;
            }

            encTicksToBox = robot.motorLeft.getCurrentPosition();

            telemetry.addData("Inches traveled before box:", robot.convertTicksToInches(encTicksToBox));
            telemetry.addData("Number of lines crossed", numOfWhiteLines);
            telemetry.update();
            idle();
        }

        robot.stop();

        robot.setRobotSpeed(-HardwareDriveBot.SLOW_POWER);

        while (robot.motorLeft.getCurrentPosition() > 0) {

            idle();
        }

        robot.stop();

        double inches = robot.convertTicksToInches(encTicksToBox);

        System.out.println("Low thresholds: " + new ColorData(lowThresholds.getRed(), lowThresholds.getBlue(), lowThresholds.getGreen(), lowThresholds.getAlpha()));
        System.out.println("High thresholds: " + new ColorData(highThresholds.getRed(), highThresholds.getBlue(), highThresholds.getGreen(), highThresholds.getAlpha()));

//        System.out.println("Low thresholds: " + new ColorData(lowRedThreshold, lowBlueThreshold, lowGreenThreshold, lowAlphaThreshold));
//        System.out.println("High thresholds: " + new ColorData(highRedThreshold, highBlueThreshold, highGreenThreshold, highAlphaThreshold));

        Log.i("Total Inches To Box", Double.toString(inches));

        telemetry.update();

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

        if (currentColorData.numOfColorsAboveThreshold(highThresholds.getRed() - wiggleRoomForSensors, highThresholds.getBlue() - wiggleRoomForSensors,
                highThresholds.getGreen() - wiggleRoomForSensors, highThresholds.getAlpha() - wiggleRoomForSensors) == 4 && overTape == false) {

            overTape = true;
            addToNumOfWhiteLines = true;
        }

        if (currentColorData.numOfColorsBelowThreshold(lowThresholds.getRed() + wiggleRoomForSensors, lowThresholds.getBlue() + wiggleRoomForSensors,
                lowThresholds.getGreen() + wiggleRoomForSensors, lowThresholds.getAlpha() + wiggleRoomForSensors) == 4) {

            overTape = false;
        }

        robotPositionInfo[0] = overTape;
        robotPositionInfo[1] = addToNumOfWhiteLines;

        return robotPositionInfo;
    }
}
