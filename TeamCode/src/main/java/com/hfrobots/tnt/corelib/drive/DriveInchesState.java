/**
 Copyright (c) 2016 HF Robotics (http://www.hfrobots.com)
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

package com.hfrobots.tnt.corelib.drive;

import android.util.Log;

import com.hfrobots.tnt.corelib.control.DebouncedGamepadButtons;
import com.hfrobots.tnt.corelib.state.State;
import com.hfrobots.tnt.corelib.state.TimeoutSafetyState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * State machine state that will drive a TankDrive the given number of inches. Notice that
 * this class assumes we're using "west coast" dual-motor drive and implements it's own
 * encoder value tracking since FTC motor controllers can't handle two motors on same output
 * or run competing PID loops (no way to synchronize).
 */
public class DriveInchesState extends TimeoutSafetyState {
    private final TankDrive drive;
    private final double powerLevel;
    private final double inchesToDrive;
    private boolean driveStarted = false;
    private TankDrive.SidedTargetPositions targetPositions;
    private final static long THRESHOLD_ENCODER_VALUE = 10;


    /**
     * Constructs a state machine state that will drive the TankDrive the number of inches (- is reverse)
     * at the given power level. The state will transition to the state given by setNextState() when
     * the distance is reached (within a threshold), or if the timeout is reached.
     */
    // TODO - would be nice to auto-calculate the timeout here based on distance/power
    public DriveInchesState(String name, TankDrive drive,
                            Telemetry telemetry,
                            double inchesToDrive,
                            double powerLevel,
                            long safetyTimeoutMillis) {
        super(name, telemetry, safetyTimeoutMillis);
        this.drive = drive;
        this.powerLevel = powerLevel;
        this.inchesToDrive = inchesToDrive;
    }

    @Override
    public State doStuffAndGetNextState() {
        if (!driveStarted) {
            // we need to do something here to drive using encoders, what?
            targetPositions = drive.getTargetPositionsForInchesTravel(inchesToDrive);
            drive.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive.drivePower(powerLevel, powerLevel);
            driveStarted = true;

            return this;
        }

        TankDrive.SidedTargetPositions currentPositions = drive.getCurrentPositions();

        telemetry.addData("04", "inches tl/tr/cl/cr: " + targetPositions.getLeftTargetPosition() + "/"
                + targetPositions.getRightTargetPosition() + "/" + currentPositions.getLeftTargetPosition() + "/"
                + currentPositions.getRightTargetPosition());

        if (currentPositions.getLeftTargetPosition() >= targetPositions.getLeftTargetPosition() ||
                currentPositions.getRightTargetPosition() >= targetPositions.getRightTargetPosition()) {
            Log.d("VV", "Encoder positions met or exceeded - stopping drive");
            stopDriving();

            return nextState;
        }

        long leftDifference = Math.abs(targetPositions.getLeftTargetPosition() - currentPositions.getLeftTargetPosition());
        long rightDifference = Math.abs(targetPositions.getRightTargetPosition() - currentPositions.getRightTargetPosition());

        if (leftDifference < THRESHOLD_ENCODER_VALUE && rightDifference < THRESHOLD_ENCODER_VALUE) {
                Log.d("VV", "Encoder positions reached w/in threshold - stopping drive");
                stopDriving();

                return nextState;
        }

        if (isTimedOut()) {
            Log.e("VV", "Drive inches state timed out- stopping drive");
            stopDriving();

            return nextState;
        }

        return this;
    }

    @Override
    public void resetToStart() {
        super.resetToStart();
        driveStarted = false;
        targetPositions = null;
    }

    @Override
    public void liveConfigure(DebouncedGamepadButtons buttons) {

    }

    private void stopDriving() {
        // (1) Stop the motors
        drive.drivePower(0, 0);
        // (2) "reset" the motors/drive train to a "normal" state, which is?
        drive.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
