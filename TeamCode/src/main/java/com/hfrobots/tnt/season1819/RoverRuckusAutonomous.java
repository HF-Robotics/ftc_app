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

import com.hfrobots.tnt.corelib.Constants;
import com.hfrobots.tnt.corelib.control.DebouncedGamepadButtons;
import com.hfrobots.tnt.corelib.state.State;
import com.hfrobots.tnt.corelib.state.TimeoutSafetyState;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

@Autonomous(name="00 RoverRuckus Auto")
@SuppressWarnings("unused")
public class RoverRuckusAutonomous extends RoverRuckusHardware {

    private State currentState = null;

    // The routes our robot knows how to do
    private enum Routes {
        Only_Option("Our only option");

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

    // Which alliance are we? (the robot is programmed from the point-of-view of the red alliance
    // but we can also have it run the blue one if selected

    private Constants.Alliance currentAlliance = Constants.Alliance.RED;

    private int initialDelaySeconds = 0;

    public RoverRuckusAutonomous() {
        imuNeeded = false; // for now...
    }

    @Override
    public void init() {
        super.init();
        setDefaults();

        for (DcMotor motor : mecanumDrive.motors) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    @Override
    public void start() {
        super.start();
        logBatteryState("Auto.start()");
    }

    @Override
    public void stop() {
        super.stop();
        logBatteryState("Auto.stop()");
    }

    private void setDefaults() {
        currentAlliance = Constants.Alliance.RED;
        selectedRoutesIndex = 0;
        initialDelaySeconds = 0;
    }

    private boolean configLocked = false;

    // Called repeatedly after init button has been pressed and init() has completed (we think)
    @Override
    public void init_loop() {
        // FIXME: Post league-meet 1, discuss homing the linear actuator here...if we do, homing
        // needs some timeouts...
        
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
        telemetry.addData("02", "Task: %s", possibleRoutes[selectedRoutesIndex].getDescription());
        telemetry.addData("03", "Delay %d sec", initialDelaySeconds);

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
            currentAlliance = Constants.Alliance.RED;
        } else if (driverXBlueButton.getRise()) {
            currentAlliance = Constants.Alliance.BLUE;
        }
    }

    @Override
    public void loop() {
        long cycleStartTimeMs = System.currentTimeMillis();

        try {
            if (currentState == null) {
                /* We have not configured the state machine yet, do so from the options
                 selected during init_loop() */

                final State selectedState;
                Routes selectedRoute = possibleRoutes[selectedRoutesIndex];

                switch (selectedRoute) {
                    case Only_Option:
                        selectedState = descendOnly();
                        break;
                    default:
                        selectedState = newDoneState("Default done");
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
                Log.d(LOG_TAG, String.format("State transition from %s to %s", currentState.getClass()
                        + "(" + currentState.getName() + ")", nextState.getClass() + "(" + nextState.getName() + ")"));
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

        long cycleStopTimeMs = System.currentTimeMillis();
        Log.d(Constants.LOG_TAG, "Cycle time (ms): " + (cycleStopTimeMs - cycleStartTimeMs));
    }

    /**
     * Creates an instance of the "delay" state which waits the number of seconds before
     * advancing to the next state
     */
    protected State newDelayState() {
        return newDelayState("start delay", initialDelaySeconds);
    }

    protected State descendOnly() {
        State initialState = new DescenderState(telemetry);

        MecanumStrafeDistanceState awayFromLanderOne = new MecanumStrafeDistanceState(
                "away from lander one", telemetry, mecanumDrive, 1.0,
                TimeUnit.SECONDS.toMillis(5));
        initialState.setNextState(awayFromLanderOne);

        MecanumDriveDistanceState offTheHook = new MecanumDriveDistanceState("off the hook",
                telemetry, mecanumDrive, 1.5, TimeUnit.SECONDS.toMillis(5));
        awayFromLanderOne.setNextState(offTheHook);

        MecanumStrafeDistanceState awayFromLanderFinal = new MecanumStrafeDistanceState(
                "away from lander Final", telemetry, mecanumDrive, 1.5,
                TimeUnit.SECONDS.toMillis(5));
        offTheHook.setNextState(awayFromLanderFinal);
        awayFromLanderFinal.setNextState(newDoneState("done"));

        return initialState;
    }

    class DescenderState extends TimeoutSafetyState {
        public DescenderState(final Telemetry telemetry) {
            super("descending", telemetry, TimeUnit.SECONDS.toMillis(15));
            //FixMe how long do we do this?
            // Not 27 didn't allow for run of the off the hook step. I was watching the robot and
            // it keep descending during auto and didn't leave enough time to descend. I think it
            // would be ok to go into drive state if we reach time ou bc if we have not reach the
            // ground then we won't go anywhere, if we have then we will still get full auto.
                // I didn't see the wheels move during off auto's. check logs.
                // - Lauren

            //tried 15 may still not be right - lauren 11/18
                //we removed tape on acDc mech and it worked w/ this code - lauren l8r 11/18
        }

        private boolean stateStarted = false;


        @Override
        public State doStuffAndGetNextState() {
            if (!stateStarted) {
                ascenderDescender.extendToMax();

                stateStarted = true;

                return this;
            } else {
                if (isTimedOut()) {
                    Log.d(Constants.LOG_TAG, "Descender timed out after " + safetyTimeoutMillis + " ms");

                    ascenderDescender.stopMoving();

                    return newDoneState("Descender timed out");

                } else if (ascenderDescender.hasExtended()) {
                    ascenderDescender.stopMoving();

                    return nextState;
                } else {
                    return this;
                }
            }
        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {
            // do nothing
        }
    }
}