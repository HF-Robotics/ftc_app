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

import com.hfrobots.tnt.corelib.control.DebouncedGamepadButtons;
import com.hfrobots.tnt.corelib.drive.DriveInchesState;
import com.hfrobots.tnt.corelib.drive.DriveUntilTouchState;
import com.hfrobots.tnt.corelib.drive.GyroTurnState;
import com.hfrobots.tnt.corelib.drive.Turn;
import com.hfrobots.tnt.corelib.state.State;
import com.hfrobots.tnt.corelib.state.StateMachine;
import com.hfrobots.tnt.corelib.units.RotationalDirection;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

@Autonomous(name="VV Beacons")
@SuppressWarnings("unused")
public class VelocityVortexBeacons extends VelocityVortexHardware {

    private static final double POWER_LEVEL = 0.4;

    private static final String LOG_TAG = "TNT Auto";
    private StateMachine stateMachine = null;

    // The routes our robot knows how to do
    private enum Routes {
        CLAIM_CLOSEST_BEACON("Claim closest beacon");

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
            if (stateMachine == null) {
                /* We have not configured the state machine yet, do so from the options
                 selected during init_loop() */

                Routes selectedRoute = possibleRoutes[selectedRoutesIndex];

                switch (selectedRoute) {
                    case CLAIM_CLOSEST_BEACON:
                        stateMachine = claimClosestBeacon();
                        break;
                    default:
                        throw new IllegalArgumentException("Invalid route selected");
                }

                if (initialDelaySeconds != 0) {
                    stateMachine.addStartDelay(initialDelaySeconds);
                }
            }

            stateMachine.doOneStateLoop();

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
    private State newDoneState(String name) {
        return new State(name, telemetry) {
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

            @Override
            public void resetToStart() {
                issuedStop = false;
            }

            @Override
            public void liveConfigure(DebouncedGamepadButtons buttons) {

            }
        };
    }

    private StateMachine claimClosestBeacon() {
        StateMachine stateMachine = new StateMachine(telemetry);

        // (1) Drive forward until touch sensor is pressed
        State moveForwardState = new DriveUntilTouchState("Forward until touch sensor", drive, telemetry, beaconTouchSensor, 0.25, 3000);

        stateMachine.setFirstState(moveForwardState);

        // (2) Detect beacon color and press button
        State detectAndPressBeaconState = new BeaconPusherState(telemetry);
        moveForwardState.setNextState(detectAndPressBeaconState);
        State doneState = newDoneState("Claim closest beacon done");
        detectAndPressBeaconState.setNextState(doneState);
        stateMachine.addNewState(detectAndPressBeaconState);
        stateMachine.addNewState(doneState);

        return stateMachine;
    }

    class BeaconPusherState extends State {
        public BeaconPusherState(Telemetry telemetry) {
            super("BeaconPusher", telemetry);
        }

        @Override
        public State doStuffAndGetNextState() {
            int blueColorReading = beaconColorSensor.blue();
            int redColorReading = beaconColorSensor.red();
            int greenColorReading = beaconColorSensor.green();

            // TODO: We should attempt to get the color a few times until we're satisfied
            // we have a good reading (it's registering not blue, but not some other wacky color)

            // Better not get this wrong, it's 30 points for the other team if you do

            // TODO: What can/should we do to determine if color sensor isn't sensing correctly? If

            if (blueColorReading >= 5) {
                Log.d("VV", "Detected blue on detector side (" + blueColorReading + ")");
                // We detect blue (not red), the color sensor is on the left side,
                // so depending on what it sees, do the alliance-specific action
                if (currentAlliance == Alliance.RED) {
                    //pick the right side (not the wrong one)
                    telemetry.addData("01", "Pushing right side of beacon");
                    Log.d("VV", "Pushing right side of beacon");
                } else {
                    telemetry.addData("01", "Pushing left side of beacon");
                    Log.d("VV", "Pushing left side of beacon");
                }
            } else {
                Log.d("VV", "Detected not blue on detector side (" + blueColorReading + ")");
                if (currentAlliance == Alliance.RED) {
                    //pick the right side (not the wrong one)
                    telemetry.addData("01", "Pushing left side of beacon");
                    Log.d("VV", "Pushing left side of beacon");
                } else {
                    telemetry.addData("01", "Pushing right side of beacon");
                    Log.d("VV", "Pushing right side of beacon");
                }
            }

            return this;
        }

        @Override
        public void resetToStart() {

        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }
    }
}
