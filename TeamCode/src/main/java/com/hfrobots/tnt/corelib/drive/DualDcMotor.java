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

import com.hfrobots.tnt.corelib.units.RotationalDirection;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DualDcMotor implements ExtendedDcMotor {
    private final ExtendedDcMotor firstMotor;

    private final ExtendedDcMotor secondMotor;

    public static DualDcMotor dualNeverest20(HardwareMap hardwareMap, String firstMotorName, String secondMotorName) {
        ExtendedDcMotor firstMotor = NinjaMotor.asNeverest20(hardwareMap.dcMotor.get(firstMotorName));
        ExtendedDcMotor secondMotor = NinjaMotor.asNeverest20(hardwareMap.dcMotor.get(secondMotorName));

        return new DualDcMotor(firstMotor, secondMotor);
    }

    public DualDcMotor(ExtendedDcMotor primaryMotor, ExtendedDcMotor secondaryMotor) {
        this.firstMotor = primaryMotor;
        this.secondMotor = secondaryMotor;
    }

    @Override
    public void setMaxSpeed(int encoderTicksPerSecond) {
        firstMotor.setMaxSpeed(encoderTicksPerSecond);
        secondMotor.setMaxSpeed(encoderTicksPerSecond);
    }

    @Override
    public int getMaxSpeed() {
        return firstMotor.getMaxSpeed();
    }

    @Override
    public DcMotorController getController() {
        return firstMotor.getController(); // what do we do here, do we force/assume they use the same controller?
    }

    @Override
    public int getPortNumber() {
        return 0; // what do we do here?
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        firstMotor.setZeroPowerBehavior(zeroPowerBehavior);
        secondMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {

        return firstMotor.getZeroPowerBehavior();
    }

    @Override
    public void setPowerFloat() {
        firstMotor.setPowerFloat();
        secondMotor.setPowerFloat();
    }

    @Override
    public boolean getPowerFloat() {
        return firstMotor.getPowerFloat();
    }

    @Override
    public void setTargetPosition(int position) {
        // what do we do here, each motor will have different current positions

    }

    @Override
    public int getTargetPosition() {
        return 0;
    }

    @Override
    public boolean isBusy() {
        return firstMotor.isBusy() || secondMotor.isBusy();
    }

    @Override
    public int getCurrentPosition() {
        return 0; // what do we do here, each motor will have different current positions
    }

    @Override
    public void setMode(RunMode mode) {
        firstMotor.setMode(mode);
        secondMotor.setMode(mode);
    }

    @Override
    public RunMode getMode() {
        return firstMotor.getMode();
    }

    @Override
    public void setDirection(Direction direction) {
        firstMotor.setDirection(direction);
        secondMotor.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        if (!firstMotor.getDirection().equals(secondMotor.getDirection())) {
            throw new RuntimeException("Directions are different between paired motors!");
        }

        return firstMotor.getDirection();
    }

    @Override
    public void setPower(double power) {
        firstMotor.setPower(power);
        secondMotor.setPower(power);
    }

    @Override
    public double getPower() {
        if (firstMotor.getPower() != secondMotor.getPower()) {
            throw new RuntimeException("Power is different between paired motors!");
        }

        return firstMotor.getPower();

    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return null; // what do we do here?
    }

    @Override
    public String getConnectionInfo() {
        return null; // what do we do here?
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        firstMotor.resetDeviceConfigurationForOpMode();
        secondMotor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        firstMotor.close();
        secondMotor.close();
    }

    @Override
    public RotationalDirection getMotorNativeDirection() {
        return firstMotor.getMotorNativeDirection();
    }

    @Override
    public int getEncoderCountsPerRevolution() {
        return firstMotor.getEncoderCountsPerRevolution();
    }

    @Override
    public void setRelativeTargetPosition(int position) {
        firstMotor.setRelativeTargetPosition(position);
        secondMotor.setRelativeTargetPosition(position);
    }

    @Override
    public int getCurrentRelativePosition() {
        return firstMotor.getCurrentRelativePosition();
    }
}
