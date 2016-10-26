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

package com.hfrobots.tnt.season1617;

import com.hfrobots.tnt.corelib.state.State;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="VV Auto")
@SuppressWarnings("unused")
public class VelocityVortexAutonomous extends VelocityVortexHardware {

    private State currentState;

    private State DoneState = new State(telemetry) {
        private boolean issuedStop = false;

        @Override
        public State doStuffAndGetNextState() {
            if (!issuedStop) {
                drive.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                drive.drivePower(0, 0);

                issuedStop = true;
            }

            return this;
        }
    };

    @Override
    public void init() {
        super.init();
    }


    @Override
    public void loop() {
        State nextState = currentState.doStuffAndGetNextState();

        if (nextState != currentState) {
            // log and/or telemetry?
        }

        currentState = nextState;
        telemetry.update(); // send all telemetry to the drivers' station
    }

    /*
        (1) Start at position #1
        (2) Move forward 25 inches
        (3) Rotate 77.5 deg CCW
        (4) Move forward 13 inches
        (5) Rotate 40 deg CCW
        (6) Move forward 38.5 inches
        (7) Done
     */
    private State parkOnRamp1() {
        return null;
    }

    /*
        (1) Start at position #2
        (2) Move forward 14.5 inches
        (3) Rotate 57.5 deg CCW
        (4) Move forward 39 inches
        (5) Rotate 70 deg CCW
        (6) Move forward 40.5 inches
        (7) Done
     */
    private State parkOnRamp2() {
        return null;
    }

    /*
        (1) Start at position #3
        (2) Move forward 3.5 inches
        (3) Rotate 45 degrees CCW
        (4) Move forward 18 inches
        (5) Rotate 22.5 degrees CCW
        (6) Move forward 52 inches
        (7) Rotate 67.5 degrees CCW
        (8) Move forward 34 inches
        (9) end
     */
    private State parkOnRampDangerWillRobinson() {
        return null;
    }

    /*
        (1) Start at position #1
        (2) Move forward 51 inches
        (3) Done

     */
    private State parkOnVortex1() {
        return null;
    }

    /*
        (1) Start at position #2
        (2) Move forward 48.5 inches
        (3) Done
     */
    private State parkOnVortex2() {
        return null;
    }

    /*
        (1) Start at position #3
        (2) Move forward 3.5 inches
        (3) Rotate 45deg CCW
        (4) Move forward 58.5 inches
        (5) Done
     */
    private State parkOnVortex3() {
        return null;
    }
}
