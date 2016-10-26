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

import com.hfrobots.tnt.corelib.state.State;
import com.hfrobots.tnt.corelib.state.TimeoutSafetyState;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveInchesState extends TimeoutSafetyState {
    private final TankDrive drive;
    private final double powerLevel;
    private final double inchesToDrive;

    private boolean driveStarted = false;

    public DriveInchesState(TankDrive drive,
                            Telemetry telemetry,
                            double inchesToDrive,
                            double powerLevel,
                            long safetyTimeoutMillis) {
        super(telemetry, safetyTimeoutMillis);
        this.drive = drive;
        this.powerLevel = powerLevel;
        this.inchesToDrive = inchesToDrive;
    }

    @Override
    public State doStuffAndGetNextState() {
        if (!driveStarted) {
            // we need to do something here to drive using encoders, what?

            driveStarted = true;

            return this; // loop() has to get called again for this initialization to happen
        }

        if (false /* what is the "exit condition" for this state? */) {
            return nextState;
        }

        return this;
    }
}
