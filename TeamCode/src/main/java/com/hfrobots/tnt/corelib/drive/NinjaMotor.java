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

package com.hfrobots.tnt.corelib.drive;

import android.util.Log;

import static com.hfrobots.tnt.corelib.units.RotationalDirection.CLOCKWISE;
import static com.hfrobots.tnt.corelib.units.RotationalDirection.COUNTER_CLOCKWISE;

import com.hfrobots.tnt.corelib.units.RotationalDirection;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * A DcMotor wrapper implementing our ExtendedDcMotor interface that
 * adds required functionality (such as knowing the encoder counts). Factory methods
 * exist for the motor models/brands we use that allow programmers to "forget" which
 * way the motors rotate and how many PPR each motor has.
 */
public class NinjaMotor implements ExtendedDcMotor {

    private final int encoderCountsPerRevolution;
    private final DcMotor dcMotor;
    private int absoluteTargetPosition;
    private final RotationalDirection motorNativeDirection;

    /**
     * Creates an ExtendedDcMotor from the given FTC SDK Motor with Neverest 20 behavior
     */
    public static ExtendedDcMotor asNeverest20(DcMotor dcMotor) {
        Log.d("VV", "asNeverest20(" + (dcMotor == null ? "null" : dcMotor) + ")");

        //Fix me rotates backwards compared to other motors
        return new NinjaMotor(dcMotor, CLOCKWISE, 560);
    }

    /**
     * Creates an ExtendedDcMotor from the given FTC SDK Motor with Neverest 40 behavior
     */
    public static ExtendedDcMotor asNeverest40(DcMotor dcMotor) {

        Log.d("VV", "asNeverest40(" + (dcMotor == null ? "null" : dcMotor) + ")");


        return new NinjaMotor(dcMotor, COUNTER_CLOCKWISE, 1120);
    }

    /**
     * Creates an ExtendedDcMotor from the given FTC SDK Motor with Neverest 60 behavior
     */
    public static ExtendedDcMotor asNeverest60(DcMotor dcMotor) {

        Log.d("VV", "asNeverest60(" + (dcMotor == null ? "null" : dcMotor) + ")");

        return new NinjaMotor(dcMotor, COUNTER_CLOCKWISE, 420);
    }

    /**
     * Creates an ExtendedDcMotor from the given FTC SDK Motor with Tetrix gear motor behavior
     */
    public static ExtendedDcMotor asTetrix(DcMotor dcMotor) {

        Log.d("VV", "asTetrix20(" + (dcMotor == null ? "null" : dcMotor) + ")");

        return new NinjaMotor(dcMotor, CLOCKWISE, 1440);
    }

    /**
     * Private because we want to have code only use the factory methods
     */
    private NinjaMotor(DcMotor dcMotor, RotationalDirection motorNativeDirection, int encoderCountsPerRevolution) {
        this.encoderCountsPerRevolution = encoderCountsPerRevolution;
        this.dcMotor = dcMotor;
        this.motorNativeDirection = motorNativeDirection;
    }

    /**
     * Which way does the motor shaft rotate when plugged directly into power?
     */
    @Override
    public RotationalDirection getMotorNativeDirection() {
        return motorNativeDirection;
    }

    /**
     * How many PPR does this motor produce?
     */
    @Override
    public int getEncoderCountsPerRevolution() {
        return encoderCountsPerRevolution;
    }

    /**
     * Sets the absolute target position for this motor to current position + relativePosition
     */
    @Override
    public int setRelativeTargetPosition(int relativePosition) {
        int currentMotorPosition = dcMotor.getCurrentPosition();
        int absolutePosition = currentMotorPosition + relativePosition;
        dcMotor.setTargetPosition(absolutePosition);
        Log.d("VV", "NinjaMotor sRTP() - r/c/a" + relativePosition + "/" + currentMotorPosition + "/" + absolutePosition);

        return absolutePosition;
    }

    /**
     * Returns the current relative position from the last time setRelativeTargetPosition() was
     * called. If setRelativeTargetPosition() has never been called, this will return the same
     * value as calling getCurrentPosition().
     */
    @Override
    public int getCurrentRelativePosition() {
        return dcMotor.getCurrentPosition() - absoluteTargetPosition;
    }

    @Override
    public Manufacturer getManufacturer() {
        return dcMotor.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return dcMotor.getDeviceName();
    }

    @Override
    public int getTargetPosition() {
        return dcMotor.getTargetPosition();
    }

    @Override
    public void setPowerFloat() {
        dcMotor.setPowerFloat();
    }

    @Override
    public void setPower(double power) {
        dcMotor.setPower(power);
    }

    @Override
    public void close() {
        dcMotor.close();
    }

    @Override
    public boolean getPowerFloat() {
        return dcMotor.getPowerFloat();
    }

    @Override
    public int getCurrentPosition() {
        return dcMotor.getCurrentPosition();
    }

    @Override
    public void setMode(RunMode mode) {
        dcMotor.setMode(mode);
    }

    @Override
    public RunMode getMode() {
        return dcMotor.getMode();
    }

    @Override
    public void setMaxSpeed(int encoderTicksPerSecond) {
        dcMotor.setMaxSpeed(encoderTicksPerSecond);
    }

    @Override
    public int getMaxSpeed() {
        return dcMotor.getMaxSpeed();
    }

    @Override
    public DcMotorController getController() {
        return dcMotor.getController();
    }

    @Override
    public boolean isBusy() {
        return dcMotor.isBusy();
    }

    @Override
    public void setDirection(DcMotor.Direction direction) {
        dcMotor.setDirection(direction);
    }

    @Override
    public DcMotor.Direction getDirection() {
        return dcMotor.getDirection();
    }

    @Override
    public String getConnectionInfo() {
        return dcMotor.getConnectionInfo();
    }

    @Override
    public void setTargetPosition(int position) {
        dcMotor.setTargetPosition(position);
    }

    @Override
    public int getPortNumber() {
        return dcMotor.getPortNumber();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        dcMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return dcMotor.getZeroPowerBehavior();
    }

    @Override
    public int getVersion() {
        return dcMotor.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        dcMotor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public double getPower() {
        return dcMotor.getPower();
    }
}

