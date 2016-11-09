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

import android.util.Log;

import com.hfrobots.tnt.corelib.drive.DriveInchesState;
import com.hfrobots.tnt.corelib.drive.GyroTurnState;
import com.hfrobots.tnt.corelib.drive.Turn;
import com.hfrobots.tnt.corelib.state.State;
import com.hfrobots.tnt.corelib.units.RotationalDirection;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="VV Auto")
@SuppressWarnings("unused")
public class VelocityVortexAutonomous extends VelocityVortexHardware {

    private static final String LOG_TAG = "TNT Auto";
    private State currentState = null;

    // The routes our robot knows how to do
    private enum Routes { PARK_ON_RAMP_1, PARK_ON_RAMP_2, PARK_ON_RAMP_DANGEROUS,
        PARK_ON_VORTEX_1, PARK_ON_VORTEX_2, PARK_ON_VORTEX_3}

    private int selectedRoutesIndex = 0;

    private Routes[] possibleRoutes = Routes.values();

    private enum Alliance { RED, BLUE };

    // Which alliance are we? (the robot is programmed from the point-of-view of the red alliance
    // but we can also have it run the blue one if selected

    private Alliance currentAlliance = Alliance.RED;

    private int initialDelaySeconds = 0;

    @Override
    public void init() {
        super.init();
        gyro.calibrate();
        setDefaults();
    }

    private void setDefaults() {
        currentAlliance = Alliance.RED;
        selectedRoutesIndex = 0;
        initialDelaySeconds = 0;
    }

    // Called repeatedly after init button has been pressed and init() has completed (we think)
    @Override
    public void init_loop() {
        if (ninjaGamepad == null) { // safety, need to double check whether we actually need this
            // not ready yet init() hasn't been called
            return;
        }

        // Use driver dpad up/down to select which route to run
        if (driverDpadDown.getRise()) {
            selectedRoutesIndex--;
            if (selectedRoutesIndex < 0) {
                selectedRoutesIndex = possibleRoutes.length - 1; // why?
            }
        } else if (driverDpadUp.getRise()) {
            selectedRoutesIndex++;

            if (selectedRoutesIndex > possibleRoutes.length - 1) { // why -1?
                selectedRoutesIndex = 0;
            }
        }

        // use left/right bumper to decrease/increase delay

        if (leftBumper.getRise()) {
            initialDelaySeconds -= 1;

            if (initialDelaySeconds < 0) {
                initialDelaySeconds = 0;
            }
        } else if (rightBumper.getRise()) {
            initialDelaySeconds += 1;

            if (initialDelaySeconds > 10) {
                initialDelaySeconds = 10;
            }
        }

        // Alliance selection
        if (bRedButton.getRise()) {
            currentAlliance = Alliance.RED;
        } else if (xBlueButton.getRise()) {
            currentAlliance = Alliance.BLUE;
        }

        // Force gyro recal
        if (aGreenButton.getRise()) {
            gyro.calibrate();
        }

        telemetry.addData("01", "Alliance: %s", currentAlliance);
        telemetry.addData("02", "Route: %s", possibleRoutes[selectedRoutesIndex]);
        telemetry.addData("03", "Delay %d sec", initialDelaySeconds);
        telemetry.addData("04", "Gyro calibrating: %s", Boolean.toString(gyro.isCalibrating()));

        updateTelemetry(telemetry);
    }

    @Override
    public void loop() {
        try {
            if (currentState == null) {
                /* TODO we have no configured the state machine yet, do so from the options
                 selected during init_loop() */

                currentState = parkOnRamp1(); // not correct
            }

            State nextState = currentState.doStuffAndGetNextState();

            if (nextState != currentState) {
                // We've changed states alert the driving team, log for post-match analysis
                telemetry.addData("00-State", "From %s to %s", currentState, nextState);
                Log.d(LOG_TAG, String.format("State transition from %s to %s", currentState, nextState));
            }

            currentState = nextState;
            telemetry.update(); // send all telemetry to the drivers' station
        } catch (Throwable t) {
            // Better logging than the FTC SDK provides :(
            Log.e("VV", "Exception during state machine", t);

            if (t instanceof RuntimeException) {
                throw (RuntimeException)t;
            }

            RuntimeException rte = new RuntimeException();
            rte.initCause(t);

            throw rte;
        }
    }

    /**
     * Turns are relative to being in the red alliance. Because this game is exactly
     * mirror image, to get our routes working for the blue alliance we simply need to
     * reverse the direction of the turn
     */
    private Turn adjustTurnForAlliance(Turn origTurn) {
        if (currentAlliance == Alliance.RED) {
            return origTurn;
        }

        return origTurn.invert();
    }

    /**
     * Creates an instance of the "done" state which stops the robot and should be the
     * "end" state of all of our robot's state machines
     */
    private State newDoneState() {
        return new State(telemetry) {
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
        double powerLevel = 0.4;

        // (2) drive forward 25 inches
        DriveInchesState drive25State = new DriveInchesState(drive, telemetry, 25, powerLevel, 8000L);

        // (3) turn 77.5 deg ccw
        Turn turnStep3 = adjustTurnForAlliance(new Turn(RotationalDirection.COUNTER_CLOCKWISE, 78));

        State turnStep3State = new GyroTurnState(drive,
                gyro,
                turnStep3,
                telemetry,
                powerLevel,
                20000L);
        drive25State.setNextState(turnStep3State);

        // (4) drive forward 13 inches
        DriveInchesState drive13State = new DriveInchesState(drive, telemetry, 13, powerLevel, 8000L);
        turnStep3State.setNextState(drive13State);

        // (5) turn 40 deg ccw
        Turn turnStep5 = adjustTurnForAlliance(new Turn(RotationalDirection.COUNTER_CLOCKWISE, 40));

        State turnStep5State = new GyroTurnState(drive,
                gyro,
                turnStep5,
                telemetry,
                powerLevel,
                20000L);
        drive13State.setNextState(turnStep5State);

        // (6) drive forward 38.5 inches
        DriveInchesState drive39State = new DriveInchesState(drive, telemetry, 39, powerLevel, 8000L);
        turnStep5State.setNextState(drive39State);

        // (7) Done!
        drive39State.setNextState(newDoneState());

        return drive25State;
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
        (3) Rotate 45º CCW
        (4) Move forward 58.5 inches
        (5) Done
     */
    private State parkOnVortex3() {
        return null;
    }
}
