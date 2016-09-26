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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Move2Sec - moves the robot for 2 seconds
 *
 * @author scottarmstrong, harrisonchabinsky
 */

@Autonomous(name="Move With Encoders", group="Elon")
public class MoveWithEncoder extends LinearOpMode {

    // define the robot hardware
    HardwareDriveBot robot = new HardwareDriveBot();

    @Override
    public void runOpMode() throws InterruptedException {

        // initialize the robot
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        updateEncoderData(robot.motorLeft.getCurrentPosition());

        sleep(2000); // wait to read the display

        moveRobot(0.3, 24.0);

        sleep(2000);

        // turn robot

        robot.resetEncoderData();

        robot.motorLeft.setPower(-HardwareDriveBot.SLOW_POWER);
        robot.motorRight.setPower(HardwareDriveBot.SLOW_POWER);

        int encTarget = 1500;

        waitThenStopRobot(encTarget, robot.motorRight);

        updateEncoderData(robot.motorRight.getCurrentPosition());

        sleep(4000);
    }

    public void moveRobot (double speed, double inches) {

        robot.resetEncoderData();

        double rotations = inches / (Math.PI * HardwareDriveBot.WHEEL_DIAMETER);
        int encTarget = (int) (rotations * HardwareDriveBot.ENC_ROTATION);

        robot.setMotorSpeeds(speed);

        // wait until we reach our target position
        waitThenStopRobot(encTarget, robot.motorLeft);

        System.out.println("Encoder" + robot.motorLeft.getCurrentPosition());
        Log.i("ROBOT", "Encoder" + robot.motorLeft.getCurrentPosition());
    }

    //wait until we have reached our target position
    public void waitThenStopRobot (int encTarget, DcMotor motorWithPositionToTrack) {

        boolean targetPositionMet = false;

        while (targetPositionMet == false) {

            int pos = motorWithPositionToTrack.getCurrentPosition();

            updateEncoderData(pos);

            if (pos >= encTarget) {
                targetPositionMet = true;
            }
        }

        robot.stop();
    }

    public void updateEncoderData (int position) {
        telemetry.addData("Encoder", position);
        telemetry.update();
    }
}
