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


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Provide a basic manual operational mode that controls the tank drive.
 */
@TeleOp(name="00 RR Teleop")
public class RelicRecoveryTeleop extends RelicRecoveryTelemetry

{
    protected float throttleGain = 0.3F;

    protected float throttleExponent = 3; // MUST BE AN ODD NUMBER!

    protected float throttleDeadband = 0;

    @SuppressWarnings("unused")
    public RelicRecoveryTeleop() {
    }

    /**
     * Places the robot into the initial pre-running state.
     * Robot must fit within sizing guidelines (18x18)
     * before this state completes.
     */
    @Override
    public void init() {
        super.init();
    }


    /**
     * Implement a state machine that controls the robot during
     * manual-operation.  The state machine uses gamepad input to transition
     * between states.
     *
     * The system calls this member repeatedly while the OpMode is running.
     */
    @Override public void loop ()

    {
        handleDrivingInputs();
        handleGlyphGripper();

        //
        // Send telemetry data to the driver station.
        //
        updateTelemetry(); // Update common telemetry
        updateGamepadTelemetry();

    }

    private void handleDrivingInputs() {
        double x = driversGamepad.getLeftStickX().getPosition();
        double y = -driversGamepad.getRightStickY().getPosition();
        double rot = (driversGamepad.getRightTrigger().getPosition() - driversGamepad.getLeftTrigger().getPosition());

        double xScaled = scaleThrottleValue(x);
        double yScaled = scaleThrottleValue(y);
        double rotateScaled = scaleThrottleValue(rot);

        mecanumDrive.driveCartesian(xScaled, yScaled, rotateScaled, false, 0.0);
    }

    double scaleThrottleValue(double unscaledPower) {
        return (-1 * throttleDeadband) + (1 - throttleDeadband)
                * (throttleGain * Math.pow(unscaledPower, throttleExponent)
                + (1 - throttleGain) * unscaledPower);
    }

}

