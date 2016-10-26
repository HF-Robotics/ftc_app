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

import com.hfrobots.tnt.corelib.control.DebouncedButton;
import com.hfrobots.tnt.corelib.control.NinjaGamePad;
import com.hfrobots.tnt.corelib.state.State;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ParticleLauncher extends State {
    private final DcMotor topMotor;
    private final DcMotor bottomMotor;
    private final Servo triggerServo;
    private final NinjaGamePad gamePad;
    private State currentState;

    public ParticleLauncher(Telemetry telemetry, DcMotor topMotor, DcMotor bottomMotor, Servo triggerServo, NinjaGamePad gamePad){
        //constructor always has public/private, then a list in parenthesis.
        super(telemetry);
        this.gamePad = gamePad;
        this.topMotor = topMotor;
        this.bottomMotor = bottomMotor;
        this.triggerServo = triggerServo;
        // this. means 'this instance of whatever is after the dot'
        this.bottomMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        currentState = createStateMachine();
    }

    private State createStateMachine() {
        WaitingForButtonState waitForButton = new WaitingForButtonState(telemetry, null, topMotor, bottomMotor);
        TimedWaitState waitForMotorSpeed = new TimedWaitState(telemetry, 2000);
        waitForButton.setNextState(waitForMotorSpeed);
        ServoState raiseLeverState = new ServoState(telemetry, triggerServo, 1);
        waitForMotorSpeed.setNextState(raiseLeverState);
        ServoState lowerleverState = new ServoState(telemetry, triggerServo, 1);
        waitForMotorSpeed.setNextState(lowerleverState);
        TimedWaitState waitForLoadingTime = new TimedWaitState(telemetry, 2000);
        lowerleverState.setNextState(waitForLoadingTime);
        ButtonStillPressedState buttonStillPressedState = new ButtonStillPressedState(telemetry, null);
        TimedWaitState timedButtonCheck = new TimedWaitState(telemetry, 2000);
        buttonStillPressedState.setTimeOutState(timedButtonCheck);
        buttonStillPressedState.setRaiseLeverState(raiseLeverState);
        StopMotorState stopMotorState = new StopMotorState(telemetry, topMotor, bottomMotor);
        timedButtonCheck.setNextState(stopMotorState);
        stopMotorState.setNextState(waitForButton);

        return waitForButton;
    }

    public void shootCloseDistance() {
        shoot(.25);
    }

    public void shootMediumDistance() {
        shoot(.50);
    }

    public void shootFarDistance() {
        shoot(.75);
    }

    public void shootVeryFarDistance() {
        shoot(1.00);
    }

    private void shoot(double power) {
        // everything that's common to shooting any distance
        topMotor.setPower(power);
        bottomMotor.setPower(power);

        // TODO: figure out how to wait a bit before it shoots because it takes time to get to
        // it's full speed/power

        triggerServo.setPosition(1);

    }

    @Override
    public State doStuffAndGetNextState() {
        currentState = currentState.doStuffAndGetNextState();

        return this;
    }

    class WaitingForButtonState extends State {
        private final DebouncedButton button;

        private final DcMotor topMotor;

        private final DcMotor bottomMotor;

        public WaitingForButtonState(Telemetry telemetry, DebouncedButton button, DcMotor topMotor, DcMotor bottomMotor) {
            super(telemetry);
            this.button = button;
            this.topMotor = topMotor;
            this.bottomMotor = bottomMotor;
        }

        @Override
        public State doStuffAndGetNextState() {
            // TODO: This needs de-bounced
            if (button.getRise()) {
                topMotor.setPower(1);
                bottomMotor.setPower(1);
                return nextState;
            }

            return this;
        }
    }

    class StopMotorState extends State {

        private final DcMotor topMotor;

        private final DcMotor bottomMotor;

        public StopMotorState(Telemetry telemetry, DcMotor topMotor, DcMotor bottomMotor) {
            super(telemetry);
            this.topMotor = topMotor;
            this.bottomMotor = bottomMotor;
        }

        @Override
        public State doStuffAndGetNextState() {
            topMotor.setPower(0);
            bottomMotor.setPower(0);

            return nextState;
        }
    }


    class ServoState extends State {
        private Servo servo;

        private double servoPosition;

        public ServoState(Telemetry telemetry, Servo servo, double servoPosition) {
            super(telemetry);
            this.servo = servo;
            this.servoPosition = servoPosition;
        }

        @Override
        public State doStuffAndGetNextState() {
            return nextState;
        }
    }


    class TimedWaitState extends State {
        private long startTime;

        private long waitMillis;

        public TimedWaitState(Telemetry telemetry, long waitMillis) {
            super(telemetry);
            this.waitMillis = waitMillis;
        }

        @Override
        public State doStuffAndGetNextState() {
            if (startTime == 0) {
                startTime = System.currentTimeMillis();

                return this;
            }

            long now = System.currentTimeMillis();
            long elapsedTimeMillis = now - startTime;

            if (elapsedTimeMillis > waitMillis) {
                return nextState;
            } else {
                return this;
            }
        }
    }

    class ButtonStillPressedState extends State {
        private final Gamepad gamepad;

        private State raiseLeverState;

        public void setTimeOutState(State timeOutState) {
            this.timeOutState = timeOutState;
        }

        private State timeOutState;

        public ButtonStillPressedState(Telemetry telemetry, Gamepad gamepad) {
            super(telemetry);
            this.gamepad = gamepad;
        }

        public void setRaiseLeverState(State raiseLeverState) {
            this.raiseLeverState = raiseLeverState;
        }

        @Override
        public State doStuffAndGetNextState() {
            // TODO: This needs de-bounced
            if (gamepad.a) {
                return raiseLeverState;
            }

            return timeOutState;
        }
    }

}
