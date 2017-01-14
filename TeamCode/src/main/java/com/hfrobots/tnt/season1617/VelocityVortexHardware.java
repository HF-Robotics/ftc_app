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

import com.hfrobots.tnt.corelib.control.DebouncedButton;
import com.hfrobots.tnt.corelib.control.DebouncedGamepadButtons;
import com.hfrobots.tnt.corelib.control.NinjaGamePad;
import com.hfrobots.tnt.corelib.control.OnOffButton;
import com.hfrobots.tnt.corelib.control.RangeInput;
import com.hfrobots.tnt.corelib.control.RangeInputButton;
import com.hfrobots.tnt.corelib.drive.DriveTrain;
import com.hfrobots.tnt.corelib.drive.DualDcMotor;
import com.hfrobots.tnt.corelib.drive.ExtendedDcMotor;
import com.hfrobots.tnt.corelib.drive.Gear;
import com.hfrobots.tnt.corelib.drive.NinjaMotor;
import com.hfrobots.tnt.corelib.drive.TankDrive;
import com.hfrobots.tnt.corelib.drive.Wheel;
import com.hfrobots.tnt.corelib.state.DelayState;
import com.hfrobots.tnt.corelib.state.State;
import com.hfrobots.tnt.corelib.units.RotationalDirection;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Iterator;

public abstract class VelocityVortexHardware extends OpMode {

    protected NinjaGamePad driversGamepad;

    protected NinjaGamePad operatorsGamepad;

    protected RangeInput driverLeftStickX;

    protected RangeInput driverLeftStickY;

    protected DebouncedButton collectorToggleButton;

    protected DebouncedButton collectorReverseToggleButton;

    protected DebouncedButton driverDpadUp;

    protected DebouncedButton driverDpadDown;

    protected DebouncedButton driverDpadLeft;

    protected DebouncedButton driverDpadRight;

    protected DebouncedButton driverXBlueButton;

    protected DebouncedButton driverBRedButton;

    protected DebouncedButton driverYYellowButton;

    protected DebouncedButton driverAGreenButton;

    protected DebouncedButton driverRightBumper;

    protected DebouncedButton driverLeftBumper;

    protected DebouncedButton liftUnlockButton;

    protected DebouncedButton liftLockButton;

    protected ExtendedDcMotor collectorMotor;

    protected TankDrive drive;

    protected DcMotor topParticleShooter;

    protected DcMotor bottomParticleShooter;

    protected ModernRoboticsI2cGyro gyro;

    protected OnOffButton shooterTrigger;
    protected ExtendedDcMotor leftMotor1;
    protected ExtendedDcMotor rightMotor1;

    protected OnOffButton liftSafety;

    protected RangeInput liftThrottle;

    protected DcMotor liftMotor;

    protected OnOffButton brakeNoBrake;

    protected OnOffButton halfSpeed;

    protected OnOffButton directionFlip;

    protected DebouncedButton lockButton;

    protected DebouncedButton unlockButton;

    protected Servo liftLockServo;

    protected Servo forkTiltServo;

    protected final double FORK_TILT_SERVO_REST_POSITION = 0.89;

    protected double forkTiltServoPosition = FORK_TILT_SERVO_REST_POSITION;

    protected VoltageSensor voltageSensor;

    protected ModernRoboticsI2cColorSensor beaconColorSensor;

    protected OpticalDistanceSensor lineOdsSensor;

    protected TouchSensor beaconTouchSensor;
    protected boolean shooterOn = false; // track state to not log every time through loop()
    protected boolean shooterReverse = false;

    protected OnOffButton particleShooterBouncy;

    protected DebouncedButton particleShooterDebounced;

    /**
     * Initialize the hardware ... this class requires the following hardware map names
     *
     * DcMotor name="topParticleShooter"
     * DcMotor name="bottomParticleShooter"
     *
     * DcMotor name="leftDrive1"
     * DcMotor name="leftDrive2"
     *
     * DcMotor name="rightDrive1"
     * DcMotor name="rightDrive2"
     *
     * DcMotor name="collectorMotor"
     * DcMotor name="liftMotor"
     */
    @Override
    public void init() {

        try {
            setupDriverControls();
            setupOperatorControls();
            setupBeaconSensors();
            setupLineFollower();

            liftLockServo = hardwareMap.servo.get("liftLockServo");
            forkTiltServo = hardwareMap.servo.get("forkTiltServo");
            lockForks();
            forkTiltServo.setPosition(forkTiltServoPosition);

            collectorMotor = NinjaMotor.asNeverest40(hardwareMap.dcMotor.get("collectorMotor"));

            leftMotor1 = NinjaMotor.asNeverest20(hardwareMap.dcMotor.get("leftDrive1"));
            ExtendedDcMotor leftMotor2 = NinjaMotor.asNeverest20(hardwareMap.dcMotor.get("leftDrive2"));
            rightMotor1 = NinjaMotor.asNeverest20(hardwareMap.dcMotor.get("rightDrive1"));
            ExtendedDcMotor rightMotor2 = NinjaMotor.asNeverest20(hardwareMap.dcMotor.get("rightDrive2"));
            DualDcMotor leftMotor = new DualDcMotor(leftMotor1, leftMotor2);
            DualDcMotor rightMotor = new DualDcMotor(rightMotor1, rightMotor2);

            Wheel stealthWheel = Wheel.andyMarkStealth();
            Gear dummyGear = new Gear(1);

            DriveTrain leftDriveTrain = new DriveTrain("leftDrive", stealthWheel, RotationalDirection.COUNTER_CLOCKWISE, leftMotor, new Gear[]{dummyGear, dummyGear});
            DriveTrain rightDriveTrain = new DriveTrain("rightDrive", stealthWheel, RotationalDirection.CLOCKWISE, rightMotor, new Gear[]{dummyGear, dummyGear});
            drive = new TankDrive(leftDriveTrain, rightDriveTrain);

            topParticleShooter = hardwareMap.dcMotor.get("topParticleShooter");
            bottomParticleShooter = hardwareMap.dcMotor.get("bottomParticleShooter");
            bottomParticleShooter.setDirection(DcMotorSimple.Direction.REVERSE); // rotates opposite top

            gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");

            liftMotor = hardwareMap.dcMotor.get("liftMotor");

            Iterator<VoltageSensor> voltageSensors = hardwareMap.voltageSensor.iterator();
            if (voltageSensors.hasNext()) {
                voltageSensor = voltageSensors.next();
            }
        } catch (NullPointerException npe) {
            Log.d("VV", "NPE", npe);
            throw npe;
        }
    }

    private void setupBeaconSensors() {
        beaconTouchSensor = hardwareMap.touchSensor.get("beaconTouchSensor");
        beaconColorSensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "beaconColorSensor");
    }

    private void setupLineFollower() {
        lineOdsSensor = hardwareMap.opticalDistanceSensor.get("lineOdsSensor");
    }

    private void setupOperatorControls() {
        // Operator controls
        operatorsGamepad = new NinjaGamePad(gamepad2);
        collectorToggleButton = new DebouncedButton(operatorsGamepad.getAButton());
        liftLockButton = new DebouncedButton(operatorsGamepad.getXButton());
        liftUnlockButton = new DebouncedButton(operatorsGamepad.getYButton());
        collectorReverseToggleButton = new DebouncedButton(operatorsGamepad.getBButton());
        shooterTrigger = operatorsGamepad.getRightBumper();
        liftSafety = new RangeInputButton(operatorsGamepad.getLeftTrigger(), 0.65f);
        liftThrottle = operatorsGamepad.getLeftStickY();


    }

    private void setupDriverControls() {
        driversGamepad = new NinjaGamePad(gamepad1);
        driverLeftStickX = driversGamepad.getLeftStickX();
        driverLeftStickY = driversGamepad.getLeftStickY();

        driverDpadDown = new DebouncedButton(driversGamepad.getDpadDown());
        driverDpadUp = new DebouncedButton(driversGamepad.getDpadUp());
        driverDpadLeft = new DebouncedButton(driversGamepad.getDpadLeft());
        driverDpadRight = new DebouncedButton(driversGamepad.getDpadRight());
        driverAGreenButton = new DebouncedButton(driversGamepad.getAButton());
        driverBRedButton = new DebouncedButton(driversGamepad.getBButton());
        driverXBlueButton = new DebouncedButton(driversGamepad.getXButton());
        driverYYellowButton = new DebouncedButton(driversGamepad.getYButton());
        driverLeftBumper = new DebouncedButton(driversGamepad.getLeftBumper());
        driverRightBumper = new DebouncedButton(driversGamepad.getRightBumper());
        lockButton = new DebouncedButton(driversGamepad.getLeftStickButton());
        unlockButton = new DebouncedButton(driversGamepad.getRightStickButton());
        brakeNoBrake = driversGamepad.getRightBumper();
        halfSpeed = new RangeInputButton(driversGamepad.getLeftTrigger(), 0.65f);
        directionFlip = new RangeInputButton(driversGamepad.getRightTrigger(), 0.65f);
        particleShooterBouncy = driversGamepad.getLeftBumper();
        particleShooterDebounced = new DebouncedButton(particleShooterBouncy);
    }

    /**
     * Access whether a warning has been generated.
     */
    boolean wasWarningGenerated() {
        return warningGenerated;
    }

    /**
     * Access the warning message.
     */
    String getWarningMessage()

    {
        return warningMessage;
    }

    /**
     * Mutate the warning message by ADDING the specified message to the current
     * message; set the warning indicator to true.
     * <p/>
     * A comma will be added before the specified message if the message isn't
     * empty.
     */
    void appendWarningMessage(String exceptionMessage) {
        if (warningGenerated) {
            warningMessage += ", ";
        }
        warningGenerated = true;
        warningMessage += exceptionMessage;
    }

    /**
     * Indicate whether a message is a available to the class user.
     */
    private boolean warningGenerated = false;

    /**
     * Store a message to the user if one has been generated.
     */
    private String warningMessage;

    protected void particleCollectorOff() {
        collectorMotor.setPower(0);
    }

    protected void runParticleCollectorOutwards() {
        collectorMotor.setPower(-1);
    }

    protected void runParticleCollectorInwards() {
        collectorMotor.setPower(1);
    }

    protected void unlockForks() {
        liftLockServo.setPosition(0.09);
    }

    protected void lockForks() {
        liftLockServo.setPosition(0.647);
    }

    protected void tiltForksBack(double amountMore) {
        forkTiltServoPosition -= amountMore;

        if (forkTiltServoPosition <= 0) {
            forkTiltServoPosition = 0;
        }

        forkTiltServo.setPosition(forkTiltServoPosition);
    }

    protected void tiltForksForward(double amountMore) {
        forkTiltServoPosition -= amountMore;

        if (forkTiltServoPosition >= FORK_TILT_SERVO_REST_POSITION) {
            forkTiltServoPosition = FORK_TILT_SERVO_REST_POSITION;
        }

        forkTiltServo.setPosition(forkTiltServoPosition);
    }

    protected void logBatteryState(String opModeMethod) {
        if (voltageSensor == null) {
            Log.e("VV", String.format("No voltage sensor when logging voltage for %s"));

            return;
        }

        Log.d("VV", String.format("Robot battery voltage %5.2f at method %s()",voltageSensor.getVoltage(), opModeMethod));
    }

    protected void shooterOff() {
        if (shooterOn) {
            Log.d("VV", "Particle shooter on");
            shooterOn = false;
            shooterReverse = false;
        }
        topParticleShooter.setPower(0);
        bottomParticleShooter.setPower(0);
    }

    protected void shooterOn() {
        if (!shooterOn) {
            Log.d("VV", "Particle shooter on");
            shooterOn = true;
            shooterReverse = false;
        }
        topParticleShooter.setPower(1);
        bottomParticleShooter.setPower(1);
    }

    protected void shooterReverse() {
        if (!shooterReverse) {
            Log.d("VV", "Particle shooter on");
            shooterReverse = true;
        }
        topParticleShooter.setPower(-0.3);
        bottomParticleShooter.setPower(-0.3);
    }

    class WaitForButton extends State {
        private final OnOffButton trigger;

        public WaitForButton(OnOffButton trigger, Telemetry telemetry) {
            super("Wait for button", telemetry);
            this.trigger = trigger;
        }

        @Override
        public State doStuffAndGetNextState() {
            if (trigger.isPressed()) {
                Log.d("VV", "Shooter trigger pressed");
                return nextState;
            } else {
                return this;
            }
        }

        @Override
        public void resetToStart() {

        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }
    }

    class WaitForButtonRelease extends State {
        private final OnOffButton trigger;

        public WaitForButtonRelease(OnOffButton trigger, Telemetry telemetry) {
            super("Wait for button release", telemetry);
            this.trigger = trigger;
        }

        @Override
        public State doStuffAndGetNextState() {
            if (trigger.isPressed()) {
                return this;
            } else {
                Log.d("VV", "shooter - trigger released");
                return nextState;
            }
        }

        @Override
        public void resetToStart() {

        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }
    }

    class ShooterReverseState extends State {
        public ShooterReverseState(Telemetry telemetry) {
            super("Particle Shooter Reverse", telemetry);
        }

        @Override
        public State doStuffAndGetNextState() {
            shooterReverse();

            return nextState;
        }

        @Override
        public void resetToStart() {

        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }
    }

    class ShooterOnState extends State {
        public ShooterOnState(Telemetry telemetry) {
            super("Particle Shooter On", telemetry);
        }

        @Override
        public State doStuffAndGetNextState() {
            shooterOn();

            return nextState;
        }

        @Override
        public void resetToStart() {

        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }
    }

    class ShooterOffState extends State {
        public ShooterOffState(Telemetry telemetry) {
            super("Particle Shooter Off", telemetry);
        }

        @Override
        public State doStuffAndGetNextState() {
            shooterOff();

            return nextState;
        }

        @Override
        public void resetToStart() {

        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }
    }

    class CollectorOnState extends State {
        public CollectorOnState(Telemetry telemetry) {
            super("Collector Shooter On", telemetry);
        }

        @Override
        public State doStuffAndGetNextState() {
            runParticleCollectorInwards();

            return nextState;
        }

        @Override
        public void resetToStart() {

        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }
    }

    class CollectorOffState extends State {
        public CollectorOffState(Telemetry telemetry) {
            super("Collector Shooter Off", telemetry);
        }

        @Override
        public State doStuffAndGetNextState() {
            particleCollectorOff();
            return nextState;
        }

        @Override
        public void resetToStart() {

        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }
    }

    class ResetDelaysState extends State {
        private final DelayState[] delayStates;

        public ResetDelaysState(Telemetry telemetry, DelayState ... delayStates) {
            super("Reset delays", telemetry);
            this.delayStates = delayStates;
        }

        @Override
        public State doStuffAndGetNextState() {
            for (DelayState state : delayStates) {
                state.resetToStart();
            }

            return nextState;
        }

        @Override
        public void resetToStart() {

        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }
    }





}
