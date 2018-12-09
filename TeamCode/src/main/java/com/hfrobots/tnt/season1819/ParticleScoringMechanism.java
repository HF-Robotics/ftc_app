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

import com.hfrobots.tnt.corelib.control.DebouncedButton;
import com.hfrobots.tnt.corelib.control.DebouncedGamepadButtons;
import com.hfrobots.tnt.corelib.control.OnOffButton;
import com.hfrobots.tnt.corelib.drive.ExtendedDcMotor;
import com.hfrobots.tnt.corelib.drive.PidController;
import com.hfrobots.tnt.corelib.state.State;
import com.hfrobots.tnt.corelib.state.TimeoutSafetyState;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

public class ParticleScoringMechanism {
    private static final double ELEVATOR_POWER_LEVEL = 1;

    private static final double kP = .001;

    private static int ELEVATOR_UPPER_LIMIT_ENCODER_POS = 1700; // FIXME: Not correct, need to measure

    private static int ELEVATOR_LOWER_LIMIT_ENCODER_POS = 0; // FIXME: Is this correct?

    private static double OPEN_LOOP_ELEVATOR_POWER_LEVEL = .3;

    private OnOffButton elevatorCommandUpButton;

    private OnOffButton elevatorCommandDownButton;

    private DebouncedButton elevatorUpperLimitButton;

    private DebouncedButton elevatorLowerLimitButton;

    private DebouncedButton elevatorEmergencyStopButton;

    private ExtendedDcMotor elevatorMotor;

    private DigitalChannel upperElevatorLimit;

    private DigitalChannel lowerElevatorLimit;

    private Telemetry telemetry;

    private State currentState;

    public ParticleScoringMechanism(final OnOffButton elevatorCommandUpButton,

                                    final OnOffButton elevatorCommandDownButton,

                                    final DebouncedButton elevatorUpperLimitButton,
                                    final DebouncedButton elevatorLowerLimitButton,
                                    final DebouncedButton elevatorEmergencyStopButton,
                                    final ExtendedDcMotor elevatorMotor,
                                    final DigitalChannel upperElevatorLimit,
                                    final DigitalChannel lowerElevatorLimit,
                                    final Telemetry telemetry) {
        this.elevatorCommandUpButton = elevatorCommandUpButton;

        this.elevatorCommandDownButton = elevatorCommandDownButton;

        this.elevatorUpperLimitButton = elevatorUpperLimitButton;

        this.elevatorLowerLimitButton = elevatorLowerLimitButton;

        this.elevatorEmergencyStopButton = elevatorEmergencyStopButton;

        this.elevatorMotor = elevatorMotor;

        this.upperElevatorLimit = upperElevatorLimit;

        this.lowerElevatorLimit = lowerElevatorLimit;

        this.telemetry = telemetry;

        ElevatorGoUpperLimitState goUpperLimitState = new ElevatorGoUpperLimitState(telemetry);

        ElevatorGoLowerLimitState goLowerLimitState = new ElevatorGoLowerLimitState(telemetry);

        ElevatorDownCommandState downCommandState = new ElevatorDownCommandState(telemetry);

        ElevatorUpCommandState upCommandState = new ElevatorUpCommandState(telemetry);

        ElevatorAtUpperLimitState atUpperLimitState = new ElevatorAtUpperLimitState(telemetry);

        ElevatorAtLowerLimitState atLowerLimitState = new ElevatorAtLowerLimitState(telemetry);

        ElevatorIdleState idleState = new ElevatorIdleState(telemetry);

        goUpperLimitState.setAllTransitionToStates(goUpperLimitState, goLowerLimitState, downCommandState,
         upCommandState, atLowerLimitState, atUpperLimitState, idleState);

        goLowerLimitState.setAllTransitionToStates(goUpperLimitState, goLowerLimitState, downCommandState,
                upCommandState, atLowerLimitState, atUpperLimitState, idleState);

        downCommandState.setAllTransitionToStates(goUpperLimitState, goLowerLimitState, downCommandState,
                upCommandState, atLowerLimitState, atUpperLimitState, idleState);

        upCommandState.setAllTransitionToStates(goUpperLimitState, goLowerLimitState, downCommandState,
                upCommandState, atLowerLimitState, atUpperLimitState, idleState);

        atUpperLimitState.setAllTransitionToStates(goUpperLimitState, goLowerLimitState, downCommandState,
                upCommandState, atLowerLimitState, atUpperLimitState, idleState);

        atLowerLimitState.setAllTransitionToStates(goUpperLimitState, goLowerLimitState, downCommandState,
                upCommandState, atLowerLimitState, atUpperLimitState, idleState);

        idleState.setAllTransitionToStates(goUpperLimitState, goLowerLimitState, downCommandState,
                upCommandState, atLowerLimitState, atUpperLimitState, idleState);

        currentState = atLowerLimitState;
    }

    protected void stopElevator() {
        elevatorMotor.setPower(0);
    }

    protected void elevatorUp() {
        elevatorMotor.setPower(OPEN_LOOP_ELEVATOR_POWER_LEVEL);
    }

    protected void elevatorDown() {
        elevatorMotor.setPower(-OPEN_LOOP_ELEVATOR_POWER_LEVEL);
    }

    public void doPeriodicTask() {
        State nextState = currentState.doStuffAndGetNextState();

        if (nextState != currentState) {
            // We've changed states alert the driving team, log for post-match analysis
            telemetry.addData("00-State", "From %s to %s", currentState, nextState);
            Log.d(LOG_TAG, String.format("State transition from %s to %s", currentState.getClass()
                    + "(" + currentState.getName() + ")", nextState.getClass() + "(" + nextState.getName() + ")"));
        }

        currentState = nextState;
    }

    abstract class ElevatorBaseState extends TimeoutSafetyState {
        ElevatorGoUpperLimitState goUpperLimitState;

        ElevatorGoLowerLimitState goLowerLimitState;

        ElevatorDownCommandState downCommandState;

        ElevatorUpCommandState upCommandState;

        ElevatorAtUpperLimitState atUpperLimitState;

        ElevatorAtLowerLimitState atLowerLimitState;

        ElevatorIdleState idleState;

        ElevatorBaseState(String name, Telemetry telemetry, long timeoutMillis) {
            super(name, telemetry, TimeUnit.SECONDS.toMillis(60)); // FIXME
        }

        protected void setAllTransitionToStates(final ElevatorGoUpperLimitState goUpperLimitState,
                final ElevatorGoLowerLimitState goLowerLimitState,
                final ElevatorDownCommandState downCommandState,
                final ElevatorUpCommandState upCommandState,
                final ElevatorAtLowerLimitState atLowerLimitState,
                final ElevatorAtUpperLimitState atUpperLimitState,
                final ElevatorIdleState idleState) {
            this.goLowerLimitState = goLowerLimitState;

            this.goUpperLimitState = goUpperLimitState; // MM

            this.downCommandState = downCommandState;

            this.upCommandState = upCommandState;

            this.atLowerLimitState = atLowerLimitState;

            this.atUpperLimitState = atUpperLimitState;

            this.idleState = idleState;

            // FIXME: Assert that everything that is required, has been set! (there was a bug hiding in this code!)
            
        }

        protected State handleButtons() {
            if (elevatorCommandUpButton.isPressed()) {
                return upCommandState;
            } else if (elevatorCommandDownButton.isPressed()) {
                return downCommandState;
            } else if (elevatorUpperLimitButton.getRise()) {
                Log.d(LOG_TAG, "Sensed button press for auto upper limit");

                return goUpperLimitState;
            } else if (elevatorLowerLimitButton.getRise()) {
                Log.d(LOG_TAG, "Sensed button press for auto lower limit");

                return goLowerLimitState;
            } else if (elevatorEmergencyStopButton.getRise()) {
                stopElevator();

                return idleState;
            }

            return null;
        }
    }

    class ElevatorIdleState extends ElevatorBaseState {

        ElevatorIdleState(Telemetry telemetry) {
            super("Elev-idle", telemetry, TimeUnit.SECONDS.toMillis(60)); // FIXME
        }

        @Override
        public State doStuffAndGetNextState() {
            State fromButtonState = handleButtons();

            if (fromButtonState != null) {
                return fromButtonState;
            }

            return this; // FIXME: THis isn't correct - what should be returned if nothing has changed?
        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }
    }

    abstract class ElevatorClosedLoopState extends ElevatorBaseState {
        PidController pidController;

        boolean initialized = false;

        ElevatorClosedLoopState(String name, Telemetry telemetry, long timeoutMillis) {
            super(name, telemetry, TimeUnit.SECONDS.toMillis(60)); // FIXME
        }

        protected void setupPidController() {
            // Hint for tolerance - 1120 encoder ticks per full revolution, full revolution moves lead screw
            // 8mm...so what * distance * in mm of error is okay, then convert that to encoder ticks

            pidController = PidController.builder().setInstanceName("Scoring mechanism pid-controller")
                    .setKp(kP).setAllowOscillation(false)
                    .setTolerance(140)
                    .build();
            pidController.setOutputRange(-ELEVATOR_POWER_LEVEL, ELEVATOR_POWER_LEVEL);
        }
    }

    class ElevatorGoUpperLimitState extends ElevatorClosedLoopState {

        ElevatorGoUpperLimitState(Telemetry telemetry) {
            super("Elev-auto-up", telemetry, TimeUnit.SECONDS.toMillis(60)); // FIXME
        }

        @Override
        public State doStuffAndGetNextState() {
            Log.d(LOG_TAG, "Running closed loop state: " + getName());

            State fromButtonState = handleButtons();

            // Handle possible transitions back to open loop
            if (fromButtonState != null && fromButtonState != this) {
                Log.d(LOG_TAG, "Leaving closed loop for " + fromButtonState.getName());

                return fromButtonState;
            }

            if (!initialized) {
                Log.d(LOG_TAG, "Initializing PID for state " + getName());

                setupPidController();

                pidController.setAbsoluteSetPoint(true); // MM
                pidController.setTarget(ELEVATOR_UPPER_LIMIT_ENCODER_POS,
                        elevatorMotor.getCurrentPosition());

                initialized = true;
            }

            // closed loop control based on motor encoders

            double pidOutput = pidController.getOutput(elevatorMotor.getCurrentPosition());

            if (pidController.isOnTarget()) {
                stopElevator();

                Log.d(LOG_TAG, "Elevator reached upper target");

                return atUpperLimitState ;
            }

            if (upperElevatorLimit != null && upperElevatorLimit.getState() == false /* digital channels are inverted! */ ) {
                stopElevator();

                return atUpperLimitState;
            }

            Log.d(LOG_TAG, "Elevator setting power via PID to: " + pidOutput);

            elevatorMotor.setPower(pidOutput);

            return this;
        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }

    }


    class ElevatorGoLowerLimitState extends ElevatorClosedLoopState {

        ElevatorGoLowerLimitState(Telemetry telemetry) {
            super("Elev-auto-lower", telemetry, TimeUnit.SECONDS.toMillis(60)); // FIXME
        }

        @Override
        public State doStuffAndGetNextState() {

            Log.d(LOG_TAG, "Running closed loop state: " + getName());

            State fromButtonState = handleButtons();

            // Handle possible transitions back to open loop
            if (fromButtonState != null && fromButtonState != this) {
                Log.d(LOG_TAG, "Leaving closed loop for " + fromButtonState.getName());

                return fromButtonState;
            }

            if (!initialized) {
                Log.d(LOG_TAG, "Initializing PID for state " + getName());

                setupPidController();

                pidController.setAbsoluteSetPoint(true); // MM
                pidController.setTarget(ELEVATOR_LOWER_LIMIT_ENCODER_POS,
                        elevatorMotor.getCurrentPosition());

                initialized = true;
            }

            // closed loop control based on motor encoders

            double pidOutput = pidController.getOutput(elevatorMotor.getCurrentPosition());

            if (pidController.isOnTarget()) {
                Log.d(LOG_TAG, "Elevator reached lower target");

                stopElevator();

                return atLowerLimitState ;
            }

            if (lowerElevatorLimit != null && lowerElevatorLimit.getState() == false /* digital channels are inverted! */ ) {
                stopElevator();

                return atLowerLimitState;
            }

            elevatorMotor.setPower(pidOutput);

            return this;
        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }
    }


    class ElevatorAtUpperLimitState extends ElevatorBaseState {

        ElevatorAtUpperLimitState(Telemetry telemetry) {
            super("Elev-at-upper-limit", telemetry, TimeUnit.SECONDS.toMillis(60)); // FIXME
        }

        @Override
        public State doStuffAndGetNextState() {
            State fromButtonState = handleButtons();

            if (fromButtonState != null && fromButtonState != this &&
                    !fromButtonState.equals(goUpperLimitState) &&
                    !fromButtonState.equals(upCommandState)) {
                Log.d(LOG_TAG, getName() + " responding to buttons and transitioning to " + fromButtonState.getName());

                return fromButtonState;
            }

            Log.d(LOG_TAG, getName() + " nothing to do, remaining in same state");

            return this;
        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }
    }


    class ElevatorAtLowerLimitState extends ElevatorBaseState {

        ElevatorAtLowerLimitState(Telemetry telemetry) {
            super("Elev-at-lower-limit", telemetry, TimeUnit.SECONDS.toMillis(60)); // FIXME
        }

        @Override
        public State doStuffAndGetNextState() {
            State fromButtonState = handleButtons();

            if (fromButtonState != null &&  fromButtonState != this &&
                    !fromButtonState.equals(goLowerLimitState) &&
                    !fromButtonState.equals(downCommandState)) {
                Log.d(LOG_TAG, getName() + " responding to buttons and transitioning to " + fromButtonState.getName());

                return fromButtonState;
            }

            Log.d(LOG_TAG, getName() + " nothing to do, remaining in same state");

            return this;
        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }
    }


    class ElevatorStowedState extends ElevatorBaseState {

        ElevatorStowedState() {
            super("Elev-stowed", null, TimeUnit.SECONDS.toMillis(60)); // FIXME
        }

        @Override
        public State doStuffAndGetNextState() {
            State fromButtonState = handleButtons();

            if (fromButtonState != null) {
                return fromButtonState;
            }

            return this;
        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }
    }


    class ElevatorDownCommandState extends ElevatorBaseState{

        ElevatorDownCommandState(Telemetry telemetry) {
            super("Elev-cmd-down", telemetry, TimeUnit.SECONDS.toMillis(60)); // FIXME
        }

        @Override
        public State doStuffAndGetNextState() {
            // State fromButtonState = handleButtons();

            //if (fromButtonState != null) {
            // Log.d(LOG_TAG, "Transitioning to " + fromButtonState.getName() + " due to button press");
            //    return fromButtonState;
            //}

            if (elevatorCommandUpButton.isPressed()) {
                    return upCommandState;
            } else if (elevatorUpperLimitButton.getRise()) {
                    return goUpperLimitState;
            } else if (elevatorLowerLimitButton.getRise()){
                    return goLowerLimitState;
            } else if (elevatorEmergencyStopButton.getRise()) {
                stopElevator();

                return idleState;
            }

            if (lowerElevatorLimit != null && lowerElevatorLimit.getState() == false) {
                stopElevator();

                return atLowerLimitState;
            } else if (!elevatorCommandDownButton.isPressed()) {
                Log.d(LOG_TAG, "Elevator - down command button released");
                stopElevator();

                return idleState;
            }

            elevatorDown();

            return this;
        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }
    }

    class ElevatorUpCommandState extends ElevatorBaseState {

        ElevatorUpCommandState(Telemetry telemetry) {
            super("Elev-cmd-up", telemetry, TimeUnit.SECONDS.toMillis(60)); // FIXME
        }

        @Override
        public State doStuffAndGetNextState() {
            // State fromButtonState = handleButtons();

            // if (fromButtonState != null && fromButtonState != this) {
            //    Log.d(LOG_TAG, "Transitioning to " + fromButtonState.getName() + " due to button press");
            //    return fromButtonState;
            //}

            if (elevatorCommandDownButton.isPressed()) {
                return downCommandState;
            } else if (elevatorUpperLimitButton.getRise()) {
                return goUpperLimitState;
            } else if (elevatorLowerLimitButton.getRise()){
                return goLowerLimitState;
            } else if (elevatorEmergencyStopButton.getRise()) {
                stopElevator();

                return idleState;
            }

            if (upperElevatorLimit != null && upperElevatorLimit.getState() == false) {
                stopElevator();

                return atUpperLimitState;
            }

            if (!elevatorCommandUpButton.isPressed() /* FiXME: !elevatorCommandDownButton.isPressed()*/) {
                Log.d(LOG_TAG, "Elevator - up command button released");
                stopElevator();

                return idleState;
            }

            elevatorUp();

            return this;
        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }
    }
}
