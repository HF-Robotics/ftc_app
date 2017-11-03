/**
 Copyright (c) 2017 HF Robotics (http://www.hfrobots.com)
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 **/

package com.hfrobots.tnt.season1718;

import android.util.Log;

import com.hfrobots.tnt.corelib.control.DebouncedGamepadButtons;
import com.hfrobots.tnt.corelib.state.State;
import com.hfrobots.tnt.corelib.state.TimeoutSafetyState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

public class MecanumDriveDistanceState extends TimeoutSafetyState {
    private boolean initialized = false;

    private final MecanumDrive mecanumDrive;

    private final DcMotorSimple.Direction origLeftFrontDirection;
    private final DcMotorSimple.Direction origLeftRearDirection;
    private final DcMotorSimple.Direction origRightFrontDirection;
    private final DcMotorSimple.Direction origRightRearDirection;

    // NFNT could be cauesed by missicalculated cirumpfranc
    // ENB corection work but we don't know what is the cause of needing it
    private double inchesToDrive;

    private double powerLevel = 0.5D;

    public MecanumDriveDistanceState(String name, Telemetry telemetry, MecanumDrive mecanumDrive, double inchesToDrive, long timeoutMillis) {
        super(name, telemetry, timeoutMillis);
        // set mecanum drive, and each drive motor
        this.mecanumDrive = mecanumDrive;

        // compute "actual" distance to drive ... remember our experiment
        // NFNT could be cauesed by missicalculated cirumpfranc
        // ENB corection work but we don't know what is the cause of needing it
        this.inchesToDrive = inchesToDrive * .9D;

        this.origLeftFrontDirection = mecanumDrive.leftFrontDriveMotor.getDirection();
        this.origLeftRearDirection = mecanumDrive.leftRearDriveMotor.getDirection();
        this.origRightFrontDirection = mecanumDrive.rightFrontDriveMotor.getDirection();
        this.origRightRearDirection = mecanumDrive.rightRearDriveMotor.getDirection();
    }

    @Override
    public State doStuffAndGetNextState() {
        if (!initialized) {
            initialize();
        }  else if (isTimedOut()) {
            Log.e(LOG_TAG, "drive distance timeout reached - stopping drive");

            mecanumDrive.stopAllDriveMotors();
            resetMotorsToOriginalState();
            return nextState;
        } else {
            // (3) - Continue to check if the hub is driving the motors, stop doing stuff once done

            int curRightFrontPos = mecanumDrive.rightFrontDriveMotor.getCurrentPosition();
            int curLeftFrontPos = mecanumDrive.leftFrontDriveMotor.getCurrentPosition();
            int curRightRearPos = mecanumDrive.rightRearDriveMotor.getCurrentPosition();
            int curLeftRearPos = mecanumDrive.leftRearDriveMotor.getCurrentPosition();
            //ENB we don't see origoinal telemetry, refreshed so fast that we didn't see ir
            telemetry.addData("RFP", curRightFrontPos);
            telemetry.addData("RRP", curLeftFrontPos);
            telemetry.addData("LFP", curRightRearPos);
            telemetry.addData("LRP", curLeftRearPos);

            Log.d(LOG_TAG, "curPos RF/RR/LF/LR " + curRightFrontPos + "/" + curRightRearPos + "/" + curLeftFrontPos + "/" + curLeftRearPos);
            if (!mecanumDrive.rightFrontDriveMotor.isBusy()
                    && !mecanumDrive.rightRearDriveMotor.isBusy()
                    && !mecanumDrive.leftFrontDriveMotor.isBusy()
                    && !mecanumDrive.leftRearDriveMotor.isBusy()) {
                mecanumDrive.stopAllDriveMotors();
                telemetry.addData("END", "Reached target position");

                resetMotorsToOriginalState();
                return nextState;
            }
        }

        return this;
    }

    private void initialize() {
        initializeMotors();

        // (1) - Compute encoder counts for each wheel...remember some rotate "backwards" - later
        // get this working with "drivetrain" class

        double encoderCountPerRev = mecanumDrive.leftFrontDriveMotor.getEncoderCountsPerRevolution(); // assume all the same

        double wheelDiaInches = 4;

        Log.d(LOG_TAG, "inches to drive (adj) " + inchesToDrive);

        double encoderCountForDistance = (inchesToDrive / (wheelDiaInches * Math.PI)) * encoderCountPerRev;

        // ENB: right side rotates clockwise (backwards)

        int curRightFrontPos = mecanumDrive.rightFrontDriveMotor.getCurrentPosition();
        int curLeftFrontPos = mecanumDrive.leftFrontDriveMotor.getCurrentPosition();
        int curRightRearPos = mecanumDrive.rightRearDriveMotor.getCurrentPosition();
        int curLeftRearPos = mecanumDrive.leftRearDriveMotor.getCurrentPosition();

        int rightFrontTargetPos = curRightFrontPos + (int) encoderCountForDistance;
        mecanumDrive.rightFrontDriveMotor.setTargetPosition(rightFrontTargetPos);
        int rightRearTargetPos = curRightRearPos + (int) encoderCountForDistance;
        mecanumDrive.rightRearDriveMotor.setTargetPosition(rightRearTargetPos);
        int leftFrontTargetPos = curLeftFrontPos + (int) encoderCountForDistance;
        mecanumDrive.leftFrontDriveMotor.setTargetPosition(leftFrontTargetPos);
        int leftRearTargetPos = curLeftRearPos + (int) encoderCountForDistance;
        mecanumDrive.leftRearDriveMotor.setTargetPosition(leftRearTargetPos);

        // ENB: Need to set motor mode to run_to_position, otherwise errors happen
        for (DcMotor motor : mecanumDrive.motors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        Log.d(LOG_TAG, "encD " + encoderCountForDistance);
        Log.d(LOG_TAG, "RFP " + rightFrontTargetPos);
        Log.d(LOG_TAG, "RRP" + rightRearTargetPos);
        Log.d(LOG_TAG, "LFP " + leftFrontTargetPos);
        Log.d(LOG_TAG, "LRP" + leftRearTargetPos);
        // (2) - Cartesian drive with the correct direction

        for (DcMotor motor : mecanumDrive.motors) {
            motor.setPower(powerLevel);
        }

        initialized = true;
    }

    private void resetMotorsToOriginalState() {
        Log.d(LOG_TAG, "reset drive motors to original state");
        mecanumDrive.leftFrontDriveMotor.setDirection(origLeftFrontDirection);
        mecanumDrive.leftRearDriveMotor.setDirection(origLeftRearDirection);
        mecanumDrive.rightFrontDriveMotor.setDirection(origRightFrontDirection);
        mecanumDrive.rightRearDriveMotor.setDirection(origLeftRearDirection);
    }

    private void initializeMotors() {
        mecanumDrive.rightFrontDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        mecanumDrive.rightRearDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        mecanumDrive.leftFrontDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        mecanumDrive.leftRearDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void resetToStart() {
        super.resetToStart();
        initialize();
    }

    @Override
    public void liveConfigure(DebouncedGamepadButtons buttons) {
        if (buttons.getLeftBumper().getRise()) {
            inchesToDrive -= .25;
        } else if (buttons.getRightBumper().getRise()) {
            inchesToDrive += .25;
        }

        if (buttons.getaButton().getRise()) {
            powerLevel -= .1;
        } else if (buttons.getyButton().getRise()) {
            powerLevel += .1;
        }

        telemetry.addData("03", "power level " + powerLevel);
        telemetry.addData("04", "inches to drive " + inchesToDrive);
    }
}
