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

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * SensorLoop
 *
 * @author scottarmstrong, harrisonchabinsky
 */

@TeleOp(name="SensorLoop", group="Elon")
public class SensorLoop extends LinearOpMode {

    // define the robot hardware
    HardwareDriveBot robot = new HardwareDriveBot();

    @Override
    public void runOpMode() throws InterruptedException {

        // initialize the robot
        robot.init(hardwareMap);

        robot.resetEncoderData();

        telemetry.addData("Status", "Initializing gryoscope");
        telemetry.update();

        //robot.sensorGyro.calibrate();

//        while (robot.sensorGyro.isCalibrating()) {
//            idle();
//        }

        telemetry.addData("Status", "Initialized gyroscope");
        telemetry.update();

        waitForStart();

//        while (opModeIsActive()) {
//
//            boolean isPressed = robot.sensorTouch.isPressed();
//            String strTouch = isPressed ? "PRESSED" : "released";
//
//            isPressed =robot.sensorLegoTouch.isPressed();
//
//            double light = robot.sensorLegoLight.getLightDetected();
//
//            double distance = robot.sensorUltraSonic.getUltrasonicLevel();
//
//            double heading = robot.sensorGyro.getHeading();
//
//            telemetry.addData("light", String.format("%.3f", light));
//            telemetry.addData("distance", String.format("%3f",distance));
//
//            int red = robot.sensorColor.red();
//            int green = robot.sensorColor.green();
//            int blue = robot.sensorColor.blue();
//            int alpha = robot.sensorColor.alpha();
//
//            telemetry.addData("touch", strTouch);
//            telemetry.addData("color", String.format("R:%3d G:%3d B:%3d A:%3d", red, green, blue, alpha));
//            telemetry.update();
//
//            idle();
//        }

        robot.setRobotSpeed(HardwareDriveBot.SLOW_POWER);

        int encTicksToBox = 0;
        int numOfWhiteLines = 0;
        int inchesToGoBeforeCounting = 12;

        int lowRedThreshold = Integer.MAX_VALUE;
        int lowGreenThreshold = Integer.MAX_VALUE;
        int lowBlueThreshold = Integer.MAX_VALUE;
        int lowAlphaThreshold = Integer.MAX_VALUE;

        int highRedThreshold = 0;
        int highGreenThreshold = 0;
        int highBlueThreshold = 0;
        int highAlphaThreshold = 0;

        int wiggleRoomForSensors = 5;

        boolean overTape = false;
        boolean doCountLines = false;

        int encTicksToWaitBeforeCountingLines = robot.convertInchesToTicks(inchesToGoBeforeCounting);

        while (robot.sensorTouch.isPressed() == false) {

            if (doCountLines) {

                ColorData currentColorData = new ColorData(robot.sensorColor.red(), robot.sensorColor.green(), robot.sensorColor.blue(), robot.sensorColor.alpha());

                if (currentColorData.allColorsAboveThreshold(highRedThreshold - wiggleRoomForSensors, highBlueThreshold - wiggleRoomForSensors,
                        highGreenThreshold - wiggleRoomForSensors, highAlphaThreshold - wiggleRoomForSensors) == 4 && overTape == false) {

                        overTape = true;
                        numOfWhiteLines += 1;
                }

                if (currentColorData.allColorsBelowThreshold(lowRedThreshold + wiggleRoomForSensors, lowBlueThreshold + wiggleRoomForSensors,
                        lowGreenThreshold + wiggleRoomForSensors, lowAlphaThreshold + wiggleRoomForSensors) == 4) {

                    overTape = false;
                }

                telemetry.addData("Over Tape", overTape);
                telemetry.addData("Number of lines", numOfWhiteLines);

        } else {

                int red = robot.sensorColor.red();
                int green = robot.sensorColor.green();
                int blue = robot.sensorColor.blue();
                int alpha = robot.sensorColor.alpha();

                if (red < lowRedThreshold) {

                    lowRedThreshold = red;
                }

                if (red > highRedThreshold) {

                    highRedThreshold = red;
                }

                if (green < lowGreenThreshold) {

                    lowGreenThreshold = green;
                }

                if (green > highGreenThreshold) {

                    highGreenThreshold = green;
                }
                if (blue < lowBlueThreshold) {

                    lowBlueThreshold = blue;
                }

                if (blue > highBlueThreshold) {

                    highBlueThreshold = blue;
                }
                if (alpha < lowAlphaThreshold) {

                    lowAlphaThreshold = alpha;
                }

                if (alpha > highAlphaThreshold) {

                    highAlphaThreshold = alpha;
                }

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

        System.out.println("Low thresholds: " + new ColorData(lowRedThreshold, lowBlueThreshold, lowGreenThreshold, lowAlphaThreshold));
        System.out.println("High thresholds: " + new ColorData(highRedThreshold, highBlueThreshold, highGreenThreshold, highAlphaThreshold));

        Log.i("Total Inches To Box", Double.toString(inches));

        telemetry.update();

        sleep(4000);
    }

}
