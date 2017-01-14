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

import com.hfrobots.tnt.corelib.drive.CheesyDrive;
import com.hfrobots.tnt.corelib.state.DelayState;
import com.hfrobots.tnt.corelib.state.State;
import com.hfrobots.tnt.corelib.state.ToggleState;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import java.util.concurrent.TimeUnit;

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

    private CheesyDrive cheesyDrive;

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

        particleShooterState = createShooterStateMachine();

        cheesyDrive = new CheesyDrive(telemetry, drive,
                driversGamepad.getLeftStickY(), driversGamepad.getRightStickX(),
                driversGamepad.getAButton(), directionFlip, brakeNoBrake, halfSpeed);
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

    /**
     * Runs the state machine for the particle shooter
     */
    private void handleParticleShooter() {
        // FIXME Figure out how to make these work together
        //if (shooterTrigger.isPressed()) {
        //    shooterOn();
        //} else {
        //    shooterOff();
        //}
        particleShooterState = particleShooterState.doStuffAndGetNextState();
        telemetry.addData("04", "shooter - " + particleShooterState.getName());
    }

    /**
     * Runs the state machine for the collector mechanism
     */
    private void handleCollector() {
        collectorToggleState = collectorToggleState.doStuffAndGetNextState();
        collectorReverseToggleState = collectorReverseToggleState.doStuffAndGetNextState();
    }

    private boolean liftSafetyPressed = false; // only log once on each state transition

    private boolean autoUnlocked = true;

    private void handleLift() {
        if (liftSafety.isPressed()) {
            if (!liftSafetyPressed) {
                Log.d("VV", "Lift safety trigger pressed");

                liftSafetyPressed = true;
            }


            float liftThrottlePosition = liftThrottle.getPosition();

            // If the lift is going to move, and we haven't auto-unlocked, then
            // do so!
            if (Math.abs(liftThrottlePosition) >= .1 && !autoUnlocked) {
                Log.d("VV", "Auto-unlocking forks");

                unlockForks();
                autoUnlocked = true;
            }

            liftMotor.setPower(liftThrottlePosition);

            if (liftUnlockButton.getRise()) {
                Log.d("VV", "Unlocking forks");

                unlockForks();
            }

            float rightStickYPosition = operatorsGamepad.getRightStickY().getPosition();

            if (rightStickYPosition > 0) {
                //Log.d("VV", "Tilting forks back");

                tiltForksBack(rightStickYPosition * .025);
            } else if (rightStickYPosition < 0) {
                //Log.d("VV", "Tilting forks forward");

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
        cheesyDrive.handleDrive();
    }

    private void updateGamepadTelemetry() {
        telemetry.addData ("06", "GP1 Left x: " + -driverLeftStickX.getPosition());
        telemetry.addData ("07", "GP1 Left y: " + -driverLeftStickY.getPosition());
    }

    private State createShooterStateMachine() {
        State waitingForButtonPressState = new WaitForButton(particleShooterBouncy, telemetry);
        State collectorOffState = new CollectorOffState(telemetry);
        waitingForButtonPressState.setNextState(collectorOffState);

        DelayState waitForCollectorOffState = new DelayState("wait for collector stop", telemetry, 500, TimeUnit.MILLISECONDS);
        collectorOffState.setNextState(waitForCollectorOffState);

        // TODO: Perhaps run the collector backwards ever so slightly here to un-jam the particles?
        State shooterOnState = new ShooterOnState(telemetry);
        waitForCollectorOffState.setNextState(shooterOnState);

        DelayState waitForShooterSpeedState = new DelayState("wait for shooter speed", telemetry, 500, TimeUnit.MILLISECONDS);
        shooterOnState.setNextState(waitForShooterSpeedState);

        State collectorOnState = new CollectorOnState(telemetry);
        waitForShooterSpeedState.setNextState(collectorOnState);

        State waitForButtonReleaseState = new WaitForButtonRelease(particleShooterBouncy, telemetry);
        collectorOnState.setNextState(waitForButtonReleaseState);

        State shooterOffState = new ShooterOffState(telemetry);
        waitForButtonReleaseState.setNextState(shooterOffState);


        State endCollectorOffState = new CollectorOffState(telemetry);
        // FIXME: Only for practice

        State collectorBackOnState = new CollectorOnState(telemetry);
        shooterOffState.setNextState(collectorBackOnState);

        ResetDelaysState resetAllDelaysState = new ResetDelaysState(telemetry,
                waitForCollectorOffState,  waitForShooterSpeedState);
        endCollectorOffState.setNextState(resetAllDelaysState);
        // FIXME: Only for practice
        collectorBackOnState.setNextState(resetAllDelaysState);
        resetAllDelaysState.setNextState(waitingForButtonPressState); // back to the beginning!

        return waitingForButtonPressState;
    }
}