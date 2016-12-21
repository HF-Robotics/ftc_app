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
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * State machine state that will drive a TankDrive until the touch sensor is pressed.
 */
public class DriveUntilTouchState extends TimeoutSafetyState {
    private final TankDrive drive;
    private final double powerLevel;
    private final TouchSensor touchSensor;
    private boolean driveStarted = false;

    /**
     * Constructs a state machine state that will drive the TankDrive (- is reverse)
     * at the given power level until the touch sensor is pressed. The state will transition
     * to the state given by setNextState() when the touch sensor is pressed, or if the timeout
     * is reached.
     */
    public DriveUntilTouchState(String name, TankDrive drive,
                                Telemetry telemetry,
                                TouchSensor touchSensor,
                                double powerLevel,
                                long safetyTimeoutMillis) {
        super(name, telemetry, safetyTimeoutMillis);
        this.drive = drive;
        this.powerLevel = powerLevel;
        this.touchSensor = touchSensor;
    }

    @Override
    public State doStuffAndGetNextState() {
        if (!driveStarted) {
            Log.i("VV", "Drive until touch - drive started");
            drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drive.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive.drivePower(powerLevel, powerLevel);
            driveStarted = true;

            return this;
        }

        if (touchSensor.isPressed()) {
            Log.i("VV", "Drive until touch - touch sensor pressed- stopping drive");
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
