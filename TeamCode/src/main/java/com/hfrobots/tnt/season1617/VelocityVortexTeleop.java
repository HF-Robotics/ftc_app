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

import com.hfrobots.tnt.corelib.state.State;
import com.hfrobots.tnt.corelib.state.ToggleState;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Provide a basic manual operational mode that controls the tank drive.
 */
@TeleOp(name="00-VV Teleop")
@SuppressWarnings("unused")
public class VelocityVortexTeleop extends VelocityVortexHardware

{
    protected boolean useEncoders = false;

    private State collectorToggleState;

    private State collectorReverseToggleState;

    private State particleShooterState;

    /*
     * Construct the class.
     *
     * The system calls this member when the class is instantiated.
     */
    public VelocityVortexTeleop() {
    }

    /**
     * Places the robot into the initial pre-running state.
     * Robot must fit within sizing guidelines (18x18)
     * before this state completes.
     */
    @Override
    public void init() {
        super.init();

        // No "runaway robot"
        drive.drivePower(0, 0);
        liftMotor.setPower(0);
        topParticleShooter.setPower(0);
        bottomParticleShooter.setPower(0);


        if (!useEncoders) {
            drive.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            drive.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        collectorToggleState = new ToggleState("Collector inwards", telemetry, collectorToggleButton) {
            @Override
            protected void toggledOn() {
                Log.d("VV", "Collector toggled inwards(off->on)");

                runParticleCollectorInwards();
            }

            @Override
            protected void toggledOff() {
                Log.d("VV", "Collector toggled inwards(on->off)");

                particleCollectorOff();
            }
        };

        collectorReverseToggleState = new ToggleState("Collector outwards", telemetry, collectorReverseToggleButton) {
            @Override
            protected void toggledOn() {
                Log.d("VV", "Collector toggled outwards(off->on)");

                runParticleCollectorOutwards();
            }

            @Override
            protected void toggledOff() {
                Log.d("VV", "Collector toggled outwards(on->off)");

                particleCollectorOff();
            }
        };
    }

    @Override
    public void start() {
        super.start();
        logBatteryState("Teleop.start()");
    }

    @Override
    public void stop() {
        super.stop();
        logBatteryState("Teleop.stop()");
    }

    /**
     * Implement a state machine that controls the robot during
     * manual-operation.  The state machine uses gamepad input to transition
     * between states.
     * <p>
     * The system calls this member repeatedly while the OpMode is running.
     */
    @Override
    public void loop()

    {
        handleDrive();
        handleCollector();
        handleParticleShooter();
        handleLift();
        updateGamepadTelemetry();
    }

    private boolean shooterOn = false; // track state to not log every time through loop()

    /**
     * Runs the state machine for the particle shooter
     */
    private void handleParticleShooter() {
        if (shooterTrigger.isPressed()) {
            if (!shooterOn) {
                Log.d("VV", "Particle shooter on");
                shooterOn = true;
            }
            topParticleShooter.setPower(1);
            bottomParticleShooter.setPower(1);
        } else {
            if (shooterOn) {
                Log.d("VV", "Particle shooter on");
                shooterOn = false;
            }
            topParticleShooter.setPower(0);
            bottomParticleShooter.setPower(0);
        }
    }

    /**
     * Runs the state machine for the collector mechanism
     */
    private void handleCollector() {
        collectorToggleState = collectorToggleState.doStuffAndGetNextState();
        collectorReverseToggleState = collectorReverseToggleState.doStuffAndGetNextState();
    }

    private boolean liftSafetyPressed = false; // only log once on each state transition

    private void handleLift() {
        if (liftSafety.isPressed()) {
            if (!liftSafetyPressed) {
                Log.d("VV", "Lift safety trigger pressed");

                liftSafetyPressed = true;
            }

            liftMotor.setPower(liftThrottle.getPosition());

            if (liftUnlockButton.getRise()) {
                Log.d("VV", "Unlocking forks");

                unlockForks();
            } else if (liftLockButton.getRise()) {
                Log.d("VV", "Locking forks");

                lockForks();
            }

            float rightStickYPosition = operatorsGamepad.getRightStickY().getPosition();

            if (rightStickYPosition < 0) {
                Log.d("VV", "Tilting forks back");

                tiltForksBack(rightStickYPosition * .025);
            } else if (rightStickYPosition > 0) {
                Log.d("VV", "Tilting forks forward");

                tiltForksForward(rightStickYPosition * 0.25);
            }
        } else {
            if (liftSafetyPressed) {
                liftSafetyPressed = false;

                Log.d("VV", "Lift safety trigger released");
            }

            liftMotor.setPower(0);
        }
    }

    private void handleDrive() {
        //----------------------------------------------------------------------
        //
        // DC Motors
        //
        // Obtain the current values of the joystick controllers.
        //
        // Note that x and y equal -1 when the joystick is pushed all of the way
        // forward and all the way right (i.e. away from the human holder's body).
        //
        // The clip method guarantees the value never exceeds the range +-1.
        //
        // The DC motors are scaled to make it easier to control them at slower
        // speeds.
        //
        // The setPower methods write the motor power values to the DcMotor
        // class, but the power levels aren't applied until the loop() method ends.
        //

        final boolean isFloat;

        if (brakeNoBrake.isPressed()) {
            isFloat = false;
            drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            isFloat = true;
            drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        //updateThrottleParameters();

        // these scale the motor power based on the amount of input on the drive stick
        float xValue = scaleMotorPower(driverLeftStickX.getPosition());
        float yValue = -scaleMotorPower(driverLeftStickY.getPosition());

        if (directionFlip.isPressed()) {
            yValue = -yValue;
        }

        //calculate the power needed for each motor
        float leftPower = yValue + xValue;
        float rightPower = yValue - xValue;

        //clip the power values so that it only goes from -1 to 1
        leftPower = Range.clip(leftPower, -1, 1);
        rightPower = Range.clip(rightPower, -1, 1);

        if (halfSpeed.isPressed()) {
            leftPower = leftPower / 2;
            rightPower = rightPower / 2;
        }

        telemetry.addData("08", "Power (%s) G: %1.2f L/R: %5.2f / %5.2f", isFloat ? "F" : "B", gain, leftPower, rightPower);

        //set the power of the motors with the gamepad values
        drive.drivePower(leftPower, rightPower);
    }

    private void updateGamepadTelemetry() {
        telemetry.addData ("06", "GP1 Left x: " + -driverLeftStickX.getPosition());
        telemetry.addData ("07", "GP1 Left y: " + -driverLeftStickY.getPosition());
    }

    private void updateThrottleParameters() {
        if (driverDpadUp.getRise() && gain < 1) {
            gain += .1F;
            Log.d("VV", "Gain is " + gain);
        }

        if (driverDpadDown.getRise() && gain > 0) {
            gain -= .1F;
            Log.d("VV", "Gain is " + gain);
        }
    }
}