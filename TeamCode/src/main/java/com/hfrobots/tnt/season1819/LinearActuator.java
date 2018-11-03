/**
 Copyright (c) 2018 HF Robotics (http://www.hfrobots.com)
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

package com.hfrobots.tnt.season1819;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

// (Friday)
// spacer wasn't long enough, gear ground against side of drivebase
// kaylin was tired
// kaylin taught katelyn about the drill press
// need something to "kick" the robot out so it doesn't tilt (4")

// (Sunday)
// Added hard stop (but drilled hole in wrong spot)
// Experimented with magnetic limit switch, mounted magnet to linear actuator

public class LinearActuator {
    private final DcMotor motor;

    private int distanceToTravel = 23300;

    private int currentLowPosition = Integer.MIN_VALUE;

    private int currentHighPosition = Integer.MIN_VALUE;

    private final DigitalChannel lowPositionLimitSwitch;

    private boolean isHoming = false;

    private boolean hasHomed = false;

    public LinearActuator(final DcMotor motor, final DigitalChannel lowPositionLimitSwitch) {
        this.motor = motor;

        this.lowPositionLimitSwitch = lowPositionLimitSwitch;

        if (isLowerLimitReached()) {
            hasHomed = true;
            Log.d("TNT", "homed at start");
            calculateEncoderLimits();
        }
    }

    public void home() {
        Log.d("TNT", "homing"); // tell them we're homing

        isHoming = true;
    }

    // SuperFIXME: Is there a way to magically do this?

    public void doPeriodicTask() {
        if (isHoming) {
            // check limit switch
            if (isLowerLimitReached()) {
                // do what?
                hasHomed = true;
                // (1) stop homing
                isHoming = false;
                stopMoving();

                // (2) measure current low position and recompute current high position

                calculateEncoderLimits();
            } else {
                motor.setPower(-0.5);
            }
        }
    }

    private void calculateEncoderLimits() {
        currentLowPosition = motor.getCurrentPosition();
        currentHighPosition = currentLowPosition + distanceToTravel;
        // // tell everybody what values we came up with for current low position, high position

        Log.d("TNT", String.format("Linear actuator currentLowPos: %d, currentHighPos: %d",
                currentLowPosition, currentHighPosition));
    }

    private boolean isLowerLimitReached() {
        return !lowPositionLimitSwitch.getState();
    }

    public void extend(boolean ignoreLimits /* use with caution !!! */) {
        double direction = 1;
        double powerLevel = 1;

        if (!ignoreLimits && hasHomed) {
            int currentEncoderPosition = motor.getCurrentPosition();

            if (currentEncoderPosition >= currentHighPosition) {
                Log.d("TNT",
                        String.format("Linear actuator reached high limit of encoder position: %d",
                                currentEncoderPosition));
                motor.setPower(0);

                return;
            }
        }

        motor.setPower(powerLevel * direction);

    }

    public void autoExtend() {
          motor.setTargetPosition(currentHighPosition);
          motor.setPower(1);
    }

    public boolean isBusy() {
        return motor.isBusy();
    }

    public boolean isHoming() {
        return isHoming;
    }

    public void stopMoving() {
        isHoming = false;
        motor.setPower(0);
    }

    public void retract(boolean ignoreLimits /* use with caution !!! */) {
        double direction = -1;
        double powerLevel = 1;

        if (!ignoreLimits && isLowerLimitReached()) {
            stopMoving();

            if (!hasHomed) {
                hasHomed = true;
                Log.d("TNT", "Found home while calling retract()");
                calculateEncoderLimits();
            }

            return;
        }

        motor.setPower(powerLevel * direction);
    }
}
