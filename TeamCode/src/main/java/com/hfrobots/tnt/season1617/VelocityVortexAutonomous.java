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

import java.util.concurrent.TimeUnit;

@Autonomous(name="VV Auto")
@SuppressWarnings("unused")
public class VelocityVortexAutonomous extends VelocityVortexHardware {

    private static final double POWER_LEVEL = 0.4;

    private static final String LOG_TAG = "TNT Auto";
    private State currentState = null;

    // The routes our robot knows how to do
    private enum Routes { PARK_ON_RAMP_1("Park on ramp 1"),
        PARK_ON_RAMP_2("Park on ramp 2"),
        PARK_ON_RAMP_DANGEROUS("Park on ramp 'danger' "),
        PARK_ON_VORTEX_1("Park on center vortex 1"),
        PARK_ON_VORTEX_2("Park on center vortex 2"),
        PARK_ON_VORTEX_3("Park on center vortex 3");

        final String description;

        Routes(String description) {
            this.description = description;
        }

        public String getDescription() {
            return description;
        }
    }

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

    private boolean configLocked = false;

    // Called repeatedly after init button has been pressed and init() has completed (we think)
    @Override
    public void init_loop() {
        if (driversGamepad == null) { // safety, need to double check whether we actually need this
            // not ready yet init() hasn't been called
            return;
        }

        if (!configLocked) {
            doAutoConfig();

            if (lockButton.getRise()) {
                configLocked = true;
            }
        } else {
            if (unlockButton.getRise()) {
                configLocked = false;
            }
        }

        if (configLocked) {
            telemetry.addData("00", "LOCKED: Press Rt stick unlock");
        } else {
            telemetry.addData("00", "UNLOCKED: Press Lt stick lock");
        }

        telemetry.addData("01", "Alliance: %s", currentAlliance);
        telemetry.addData("02", "Route: %s", possibleRoutes[selectedRoutesIndex].getDescription());
        telemetry.addData("03", "Delay %d sec", initialDelaySeconds);
        telemetry.addData("04", "Gyro calibrating: %s", Boolean.toString(gyro.isCalibrating()));

        updateTelemetry(telemetry);
    }

    private void doAutoConfig() {
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

        if (driverLeftBumper.getRise()) {
            initialDelaySeconds -= 1;

            if (initialDelaySeconds < 0) {
                initialDelaySeconds = 0;
            }
        } else if (driverRightBumper.getRise()) {
            initialDelaySeconds += 1;

            if (initialDelaySeconds > 10) {
                initialDelaySeconds = 10;
            }
        }

        // Alliance selection
        if (driverBRedButton.getRise()) {
            currentAlliance = Alliance.RED;
        } else if (driverXBlueButton.getRise()) {
            currentAlliance = Alliance.BLUE;
        }

        // Force gyro recal
        if (driverAGreenButton.getRise()) {
            gyro.calibrate();
        }
    }

    @Override
    public void loop() {
        try {
            if (currentState == null) {
                /* We have not configured the state machine yet, do so from the options
                 selected during init_loop() */

                final State selectedState;
                Routes selectedRoute = possibleRoutes[selectedRoutesIndex];

                switch (selectedRoute) {
                    case PARK_ON_RAMP_1:
                        selectedState = parkOnRamp1();
                        break;
                    case PARK_ON_RAMP_2:
                        selectedState = parkOnRamp2();
                        break;
                    case PARK_ON_RAMP_DANGEROUS:
                        selectedState = parkOnRampDangerWillRobinson();
                        break;
                    case PARK_ON_VORTEX_1:
                        selectedState = parkOnVortex1();
                        break;
                    case PARK_ON_VORTEX_2:
                        selectedState = parkOnVortex2();
                        break;
                    case PARK_ON_VORTEX_3:
                        selectedState = parkOnVortex3();
                        break;
                    default:
                        selectedState = newDoneState();
                }

                if (initialDelaySeconds != 0) {
                    currentState = newDelayState();
                    currentState.setNextState(selectedState);
                } else {
                    currentState = selectedState;
                }
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
                    // TODO: "Hold" mode
                    drive.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    drive.drivePower(0, 0);

                    issuedStop = true;
                }

                return this;
            }
        };
    }

    /**
     * Creates an instance of the "done" state which stops the robot and should be the
     * "end" state of all of our robot's state machines
     */
    private State newDelayState() {
        return newDelayState(initialDelaySeconds);
    }

    private State newDelayState(final int numberOfSeconds) {
        return new State(telemetry) {
            private long startTime = 0;
            private long thresholdTimeMs = TimeUnit.SECONDS.toMillis(numberOfSeconds);

            @Override
            public State doStuffAndGetNextState() {
                if (startTime == 0) {
                    startTime = System.currentTimeMillis();
                    return this;
                }

                long now = System.currentTimeMillis();
                long elapsedMs = now - startTime;

                if (elapsedMs > thresholdTimeMs) {
                    return nextState;
                }

                telemetry.addData("04", "Delay: %d of %d ms", elapsedMs, thresholdTimeMs);
                return this;
            }
        };
    }

    private State parkOnRamp1() {
        // (1) Start at position #1
        // - no-op -

        // (2) Move forward 25 inches (15)
        DriveInchesState step2DriveState = new DriveInchesState(drive, telemetry, 7, POWER_LEVEL, 8000L);

        // (3) Rotate 47.5 deg CCW

        State step3TurnState = new GyroTurnState(drive,
                gyro,
                adjustTurnForAlliance(new Turn(RotationalDirection.COUNTER_CLOCKWISE, 48)),
                telemetry,
                POWER_LEVEL,
                20000L);
        step2DriveState.setNextState(step3TurnState);

        // (4) Move forward 13 inches (6)
        DriveInchesState step4DriveState = new DriveInchesState(drive, telemetry, 15, POWER_LEVEL, 8000L);
        step3TurnState.setNextState(step4DriveState);

        // (5) Rotate 90 deg CCW

        State step5TurnState = new GyroTurnState(drive,
                gyro,
                adjustTurnForAlliance(new Turn(RotationalDirection.COUNTER_CLOCKWISE, 90)),
                telemetry,
                POWER_LEVEL,
                20000L);
        step4DriveState.setNextState(step5TurnState);

        // (6) Move forward 38.5 inches (44)
        DriveInchesState step6DriveState = new DriveInchesState(drive, telemetry, 28, POWER_LEVEL, 8000L);
        step5TurnState.setNextState(step6DriveState);

        // Here is where we could shoot particles into the ramp

        DriveInchesState step7DriveState = new DriveInchesState(drive, telemetry, 10, POWER_LEVEL, 2000L);
        step7DriveState.setNextState(step7DriveState);

        step7DriveState.setNextState(newDoneState());

        return step2DriveState;
    }

    private State parkOnRamp2() {
        // (1) Start at position #2
        // - no-op -

        // (2) Move forward 14.5 inches
        DriveInchesState step2DriveState = new DriveInchesState(drive, telemetry, 14.5, POWER_LEVEL, 8000L);

        // (3) Rotate 57.5 deg CCW

        State step3TurnState = new GyroTurnState(drive,
                gyro,
                adjustTurnForAlliance(new Turn(RotationalDirection.COUNTER_CLOCKWISE, 58)),
                telemetry,
                POWER_LEVEL,
                20000L);
        step2DriveState.setNextState(step3TurnState);

        // (4) Move forward 39 inches

        DriveInchesState step4DriveState = new DriveInchesState(drive, telemetry, 39, POWER_LEVEL, 8000L);
        step3TurnState.setNextState(step4DriveState);

        // (5) Rotate 70 deg CCW
        State step5TurnState = new GyroTurnState(drive,
                gyro,
                adjustTurnForAlliance(new Turn(RotationalDirection.COUNTER_CLOCKWISE, 95)),
                telemetry,
                POWER_LEVEL,
                20000L);
        step4DriveState.setNextState(step5TurnState);


        // (6) Move forward 40.5 inches

        DriveInchesState step6DriveState = new DriveInchesState(drive, telemetry, 40.5 + 8 /* oomph */, POWER_LEVEL, 8000L);
        step5TurnState.setNextState(step6DriveState);

        // (7) Done

        step6DriveState.setNextState(newDoneState());

        return step2DriveState;
    }

    private State parkOnRampDangerWillRobinson() {
        // (1) Start at position #3
        // - no-op -

        // (2) Move forward 3.5 inches
        DriveInchesState step2DriveState = new DriveInchesState(drive, telemetry, 3.5, POWER_LEVEL, 8000L);

        // (3) Rotate 45 degrees CCW

        State step3TurnState = new GyroTurnState(drive,
                gyro,
                adjustTurnForAlliance(new Turn(RotationalDirection.COUNTER_CLOCKWISE, 45)),
                telemetry,
                POWER_LEVEL,
                20000L);
        step2DriveState.setNextState(step3TurnState);

        // (4) Move forward 18 inches (13)

        DriveInchesState step4DriveState = new DriveInchesState(drive, telemetry, 13, POWER_LEVEL, 8000L);
        step3TurnState.setNextState(step4DriveState);

        // (5) Rotate 22.5 degrees CCW

        State step5TurnState = new GyroTurnState(drive,
                gyro,
                adjustTurnForAlliance(new Turn(RotationalDirection.COUNTER_CLOCKWISE, 23)),
                telemetry,
                POWER_LEVEL,
                20000L);
        step4DriveState.setNextState(step5TurnState);


        // (6) Move forward 52 inches

        DriveInchesState step6DriveState = new DriveInchesState(drive, telemetry, 52, POWER_LEVEL, 8000L);
        step5TurnState.setNextState(step6DriveState);

        // (7) Rotate 67.5 degrees CCW

        State step7TurnState = new GyroTurnState(drive,
                gyro,
                adjustTurnForAlliance(new Turn(RotationalDirection.COUNTER_CLOCKWISE, 68)),
                telemetry,
                POWER_LEVEL,
                20000L);
        step6DriveState.setNextState(step7TurnState);


        step7TurnState.setNextState(newDoneState()); // FIXME FIXME FIXME

        // (8) Move forward 34 inches

        //DriveInchesState step8DriveState = new DriveInchesState(drive, telemetry, 34, POWER_LEVEL, 8000L);
        //step7TurnState.setNextState(step8DriveState);

        // (9) end

        //step8DriveState.setNextState(newDoneState());

        return step2DriveState;
    }

    private State parkOnVortex1() {

        // (1) Start at position #1
        // - no-op -

        // (2) Move forward 51 inches

        DriveInchesState step2DriveState = new DriveInchesState(drive, telemetry, 35, POWER_LEVEL, 8000L);

        State step3TurnState = new GyroTurnState(drive,
                gyro,
                adjustTurnForAlliance(new Turn(RotationalDirection.COUNTER_CLOCKWISE, 180)),
                telemetry,
                POWER_LEVEL,
                20000L);
        step2DriveState.setNextState(step3TurnState);
        step3TurnState.setNextState(newDoneState());
        return step2DriveState;
    }

    private State parkOnVortex2() {

        // (1) Start at position #2
        // - no-op -

        // 1.5 - 15 second delay
        State step1DelayState = newDelayState(15);

        // (2) Move forward 48.5 inches

        DriveInchesState step2DriveState = new DriveInchesState(drive, telemetry, 38.5, POWER_LEVEL, 8000L);
        step1DelayState.setNextState(step2DriveState);

        State step3TurnState = new GyroTurnState(drive,
                gyro,
                adjustTurnForAlliance(new Turn(RotationalDirection.COUNTER_CLOCKWISE, 68)),
                telemetry,
                POWER_LEVEL,
                20000L);
        step2DriveState.setNextState(step3TurnState);

        DriveInchesState step4DriveState = new DriveInchesState(drive, telemetry, 18, POWER_LEVEL, 8000L);

        step3TurnState.setNextState(step4DriveState);
        step4DriveState.setNextState(newDoneState());

        // (3) Done

        return step1DelayState;
    }

    private State parkOnVortex3() {

        // (1) Start at position #3
        // - no-op -

        // (2) Move forward 3.5 inches

        DriveInchesState step2DriveState = new DriveInchesState(drive, telemetry, 3.5, POWER_LEVEL, 8000L);

        // (3) Rotate 45º CCW

        State step3TurnState = new GyroTurnState(drive,
                gyro,
                adjustTurnForAlliance(new Turn(RotationalDirection.COUNTER_CLOCKWISE, 45)),
                telemetry,
                POWER_LEVEL,
                20000L);
        step2DriveState.setNextState(step3TurnState);

        // (4) Move forward 58.5 inches

        DriveInchesState step4DriveState = new DriveInchesState(drive, telemetry, 58.5, POWER_LEVEL, 8000L);
        step3TurnState.setNextState(step4DriveState);
        step4DriveState.setNextState(newDoneState());

        // (5) Done

        return step2DriveState;
    }
}
