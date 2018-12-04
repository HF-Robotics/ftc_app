package com.hfrobots.tnt.season1819;

import com.hfrobots.tnt.corelib.control.DebouncedButton;
import com.hfrobots.tnt.corelib.control.DebouncedGamepadButtons;
import com.hfrobots.tnt.corelib.control.OnOffButton;
import com.hfrobots.tnt.corelib.drive.PidController;
import com.hfrobots.tnt.corelib.state.State;
import com.hfrobots.tnt.corelib.state.TimeoutSafetyState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ParticleScoringMechanism {
    private OnOffButton elevatorCommandUpButton;
    private OnOffButton elevatorCommandDownButton;
    private DebouncedButton elevatorUpperLimitButton;
    private DebouncedButton elevatorLowerLimitButton;
    private DebouncedButton elevatorEmergencyStopButton;

    private DcMotor elevatorMotor;

    private DigitalChannel upperElevatorLimit;

    private DigitalChannel lowerElevatorLimit;

    protected void stopElevator() {

    }

    protected void elevatorUp() {

    }

    protected void elevatorDown() {

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
            super(null, null, 0); // FIXME
        }

        protected State handleButtons() {
            if (elevatorCommandUpButton.isPressed()) {
                return upCommandState;
            } else if (elevatorCommandDownButton.isPressed()) {
                return downCommandState;
            } else if (elevatorUpperLimitButton.getRise()) {
                return goUpperLimitState;
            } else if (elevatorLowerLimitButton.getRise()){
                return goLowerLimitState;
            } else if (elevatorEmergencyStopButton.getRise()) {
                stopElevator();

                return this; //fixme we need to stop.
            }

            return null;
        }
    }

    class ElevatorIdleState extends ElevatorBaseState {

        ElevatorIdleState() {
            super(null, null, 0); // FIXME
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

        ElevatorClosedLoopState(String name, Telemetry telemetry, long timeoutMillis) {
            super(null, null, 0); // FIXME
        }
    }

    class ElevatorGoUpperLimitState extends ElevatorClosedLoopState {

        ElevatorGoUpperLimitState() {
            super(null, null, 0); // FIXME
        }

        @Override
        public State doStuffAndGetNextState() {
            State fromButtonState = handleButtons();

            // Handle possible transitions back to open loop
            if (fromButtonState != null) {
                return fromButtonState;
            }

            // closed loop control based on motor encoders

            double pidOutput = pidController.getOutput(elevatorMotor.getCurrentPosition());

            if (pidController.isOnTarget()) {
                stopElevator();

                return atUpperLimitState ;
            }

            if (upperElevatorLimit.getState() == false /* digital channels are inverted! */ ) {
                stopElevator();

                return atUpperLimitState;
            }

            elevatorMotor.setPower(pidOutput);

            return this;
        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }

    }


    class ElevatorGoLowerLimitState extends ElevatorClosedLoopState {

        ElevatorGoLowerLimitState() {
            super(null, null, 0); // FIXME
        }

        @Override
        public State doStuffAndGetNextState() {
            State fromButtonState = handleButtons();

            // Handle possible transitions back to open loop
            if (fromButtonState != null) {
                return fromButtonState;
            }

            // closed loop control based on motor encoders

            double pidOutput = pidController.getOutput(elevatorMotor.getCurrentPosition());

            if (pidController.isOnTarget()) {
                stopElevator();

                return atLowerLimitState ;
            }

            if (lowerElevatorLimit.getState() == false /* digital channels are inverted! */ ) {
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

        ElevatorAtUpperLimitState() {
            super(null, null, 0); // FIXME
        }

        @Override
        public State doStuffAndGetNextState() {
            State fromButtonState = handleButtons();

            if (fromButtonState != null &&
                    !fromButtonState.equals(goUpperLimitState) &&
                    !fromButtonState.equals(upCommandState)) {
                return fromButtonState;
            }

            return this;
        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }
    }


    class ElevatorAtLowerLimitState extends ElevatorBaseState {

        ElevatorAtLowerLimitState() {
            super(null, null, 0); // FIXME
        }

        @Override
        public State doStuffAndGetNextState() {
            State fromButtonState = handleButtons();

            if (fromButtonState != null &&
                    !fromButtonState.equals(goLowerLimitState) &&
                    !fromButtonState.equals(downCommandState)) {
                return fromButtonState;
            }
            return this;
        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }
    }


    class ElevatorStowedState extends ElevatorBaseState {

        ElevatorStowedState() {
            super(null, null, 0); // FIXME
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

        ElevatorDownCommandState() {
            super(null, null, 0); // FIXME
        }

        @Override
        public State doStuffAndGetNextState() {
            State fromButtonState = handleButtons();

            if (fromButtonState != null) {
                return fromButtonState;
            }

            if (lowerElevatorLimit.getState() == false) {
                stopElevator();

                return atLowerLimitState;
            } else if (!elevatorCommandDownButton.isPressed()) {
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

        ElevatorUpCommandState() {
            super(null, null, 0); // FIXME
        }

        @Override
        public State doStuffAndGetNextState() {
            State fromButtonState = handleButtons();

            if (fromButtonState != null) {
                return fromButtonState;
            }

            if (upperElevatorLimit.getState() == false) {
                stopElevator();

                return atUpperLimitState;
            }

            if (!elevatorCommandDownButton.isPressed()) {
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
