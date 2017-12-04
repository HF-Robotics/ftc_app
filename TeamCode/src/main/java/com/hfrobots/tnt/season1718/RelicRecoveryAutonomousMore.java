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

package com.hfrobots.tnt.season1718;

import android.support.annotation.NonNull;
import android.util.Log;

import com.hfrobots.tnt.corelib.Constants;
import com.hfrobots.tnt.corelib.drive.Turn;
import com.hfrobots.tnt.corelib.state.State;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

import java.util.concurrent.TimeUnit;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

//@Autonomous(name="RR Auto More")
@SuppressWarnings("unused")
public class RelicRecoveryAutonomousMore extends RelicRecoveryHardware {

    private State currentState = null;

    // The routes our robot knows how to do
    private enum Routes {
        LEFT_STONE("Left stone"),
        RIGHT_STONE_RIGHT_CRYPTO("Right stone to right crypto "),
        RIGHT_STONE("Right stone");

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

    ;

    // Which alliance are we? (the robot is programmed from the point-of-view of the red alliance
    // but we can also have it run the blue one if selected

    private Constants.Alliance currentAlliance = Constants.Alliance.RED;

    private int initialDelaySeconds = 0;

    @Override
    public void init() {
        super.init();
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
        currentAlliance = Constants.Alliance.RED;
        selectedRoutesIndex = 0;
        initialDelaySeconds = 0;
    }

    private boolean configLocked = false;

    // Called repeatedly after init button has been pressed and init() has completed (we think)
    @Override
    public void init_loop() {
        // TODO: Test continually "storing" jewel sensors...

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

        handleGlyphGripper();

        telemetry.addData("01", "Alliance: %s", currentAlliance);
        telemetry.addData("02", "Route: %s", possibleRoutes[selectedRoutesIndex].getDescription());
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
        try {
            if (currentState == null) {
                /* We have not configured the state machine yet, do so from the options
                 selected during init_loop() */

                final State selectedState;
                Routes selectedRoute = possibleRoutes[selectedRoutesIndex];

                switch (selectedRoute) {
                    case LEFT_STONE:
                        selectedState = leftStoneStateMachine();
                        break;
                    case RIGHT_STONE:
                        selectedState = rightStoneStateMachine();
                        break;
                    //case RIGHT_STONE_RIGHT_CRYPTO:
                    //    selectedState = rightStoneRightCryptoStateMachine();
                    //    break;
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
     * Creates an instance of the "done" state which stops the robot and should be the
     * "end" state of all of our robot's state machines
     */
    protected State newDelayState() {
        return newDelayState("start delay", initialDelaySeconds);
    }

    /**
     * Turns are relative to being in the red alliance. Because this game is exactly
     * mirror image, to get our routes working for the blue alliance we simply need to
     * reverse the direction of the turn
     */
    private Turn adjustTurnFromBlueForCurrentAlliance(Turn origTurn) {
        if (currentAlliance == Constants.Alliance.BLUE) {
            return origTurn;
        }

        return origTurn.invert();
    }

    // This is the left stone only when on the blue alliance
    private State leftStoneStateMachine() {
        JewelMechanism.JewelMechanismDeploySensorState startState = commonJewelState();

        MecanumDriveDistanceState driveOffStoneState = new MecanumDriveDistanceState("give me a name",
                telemetry, mecanumDrive, 15, TimeUnit.SECONDS.toMillis(10));

        startState.setNextState(driveOffStoneState);

        MecanumDriveDistanceState strafeInwardState = new MecanumDriveDistanceState("STRAFE!!!!!!",
                telemetry, mecanumDrive,15, TimeUnit.SECONDS.toMillis(8));

        driveOffStoneState.setNextState(strafeInwardState);

        MecanumDriveDistanceState driveForwardFourState = new MecanumDriveDistanceState("give me a name",
                telemetry, mecanumDrive, 4, TimeUnit.SECONDS.toMillis(5));

        strafeInwardState.setNextState(driveForwardFourState);

        driveForwardFourState.setNextState(newDoneState("done"));



        //waitToStow.setNextState(newDoneState("Done"));

        return startState;
    }

    @NonNull
    private JewelMechanism.JewelMechanismDeploySensorState commonJewelState() {
        final JewelMechanism jewelMechUsed;

        if (currentAlliance.equals(Constants.Alliance.BLUE)) {
            jewelMechUsed = blueAllianceJewelMech;
        } else {
            jewelMechUsed = redAllianceJewelMech;
        }

        JewelMechanism.JewelMechanismDeploySensorState firstState = jewelMechUsed.getDeploySensorState(telemetry);
        State waitToDeploy = newDelayState("waiting for deploy", 2);
        firstState.setNextState(waitToDeploy);
        JewelMechanism.JewelMechanismDetectAndTurnWithMoreStuff detectAndTurnState
                = jewelMechUsed.getDetectAndTurnStateWithMoreStuff(telemetry, currentAlliance, mecanumDrive);
        waitToDeploy.setNextState(detectAndTurnState);
        return firstState;
    }

    private State rightStoneStateMachine() {
        State startingState = commonJewelState();

        // go forward till clear
        MecanumDriveDistanceState driveForwardState = new MecanumDriveDistanceState("give me a name",
                telemetry, mecanumDrive, 36.0, TimeUnit.SECONDS.toMillis(15));
        startingState.setNextState(driveForwardState);

        //turn conterclockwise 90 and allign w/cryptobox

        final Turn allianceAwareTurn = adjustTurnFromBlueForCurrentAlliance(new Turn(Rotation.CCW, 90));

        MecanumGyroTurnState.Builder turnBuilder = MecanumGyroTurnState.builder();
        turnBuilder.setTurn(allianceAwareTurn).setImu(imu).setMecanumDrive(mecanumDrive).setPLargeTurnCoeff(RobotConstants.P_LARGE_TURN_COEFF)
                .setPSmallTurnCoeff(RobotConstants.P_SMALL_TURN_COEFF).setName("Turn towards cryptobox")
                .setSafetyTimeoutMillis(TimeUnit.SECONDS.toMillis(15));

        MecanumGyroTurnState turnTowardsCryptoBox = turnBuilder.build();
        driveForwardState.setNextState(turnTowardsCryptoBox);

        // move foward
        MecanumDriveDistanceState driveForwardFourState = new MecanumDriveDistanceState("give me a name",
                telemetry, mecanumDrive, 4.0, TimeUnit.SECONDS.toMillis(5));
        turnTowardsCryptoBox.setNextState(driveForwardFourState);
        driveForwardFourState.setNextState(newDoneState("Done"));
        // place block
        // TODO: We need a block drop state

        return startingState;
    }

    private State StayOnStoneStateMachine() {
        return newDoneState("Right Stone, Right Crypto - TODO");
    }
}
