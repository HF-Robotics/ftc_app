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
import com.hfrobots.tnt.corelib.control.NinjaGamePad;
import com.hfrobots.tnt.corelib.control.OnOffButton;
import com.hfrobots.tnt.corelib.control.RangeInput;
import com.hfrobots.tnt.corelib.drive.DriveTrain;
import com.hfrobots.tnt.corelib.drive.DualDcMotor;
import com.hfrobots.tnt.corelib.drive.ExtendedDcMotor;
import com.hfrobots.tnt.corelib.drive.Gear;
import com.hfrobots.tnt.corelib.drive.NinjaMotor;
import com.hfrobots.tnt.corelib.drive.TankDrive;
import com.hfrobots.tnt.corelib.drive.Wheel;
import com.hfrobots.tnt.corelib.units.RotationalDirection;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

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

    protected ExtendedDcMotor collectorMotor;

    protected TankDrive drive;

    protected DcMotor topParticleShooter;

    protected DcMotor bottomParticleShooter;

    protected ModernRoboticsI2cGyro gyro;

    protected Servo conveyorServo;

    protected DebouncedButton conveyorForwardToggleButton;

    protected DebouncedButton conveyorReverseToggleButton;

    protected OnOffButton shooterTrigger;
    protected ExtendedDcMotor leftMotor1;
    protected ExtendedDcMotor rightMotor1;

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
            // Build an instance of our more advanced gamepad class

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

            // Operator controls
            operatorsGamepad = new NinjaGamePad(gamepad2);
            collectorToggleButton = new DebouncedButton(operatorsGamepad.getAButton());
            collectorReverseToggleButton = new DebouncedButton(operatorsGamepad.getBButton());
            conveyorForwardToggleButton = new DebouncedButton(operatorsGamepad.getXButton());
            conveyorReverseToggleButton = new DebouncedButton(operatorsGamepad.getYButton());
            shooterTrigger = operatorsGamepad.getRightBumper();

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

            conveyorServo = hardwareMap.get(Servo.class, "conveyorServo");
        } catch (NullPointerException npe) {
            Log.d("VV", "NPE", npe);
            throw npe;
        }
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
     * Scale the joystick input using a nonlinear algorithm.
     */
    float scaleMotorPower(float unscaledPower) {

        //
        // Ensure the values are legal.
        //
        float clippedPower = Range.clip(unscaledPower, -1, 1);

        float[] scaleFactors =
                {0.00f, 0.05f, 0.09f, 0.10f, 0.12f
                        , 0.15f, 0.18f, 0.24f, 0.30f, 0.36f
                        , 0.43f, 0.50f, 0.60f, 0.72f, 0.85f
                        , 1.00f, 1.00f
                };

        // scale changes sensitivity of joy stick by multiplying input by numbers 0 to 1
        //
        // Get the corresponding index for the given unscaled power.
        //
        int scaleIndex = (int) (clippedPower * 16.0);

        if (scaleIndex < 0) {
            scaleIndex = -scaleIndex;
        } else if (scaleIndex > 16) {
            scaleIndex = 16;
        }

        final float scaledPower;

        if (clippedPower < 0) {
            scaledPower = -scaleFactors[scaleIndex];
        } else {
            scaledPower = scaleFactors[scaleIndex];
        }

        return scaledPower;
    }

    /**
     * Indicate whether a message is a available to the class user.
     */
    private boolean warningGenerated = false;

    /**
     * Store a message to the user if one has been generated.
     */
    private String warningMessage;

}
