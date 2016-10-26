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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DriveTrain {

    private final Wheel wheel;
    private final ExtendedDcMotor motor;
    private final double gearRatio;
    private final int direction;
    private final RotationalDirection wheelRotatingForwardDirection;


    /**
     * Creates a drive train for the given wheel and gear train.
     *
     * @param wheel The wheel used in the drive train
     * @param wheelRotatingForwardDirection The direction the wheel rotates for forward motion
     * @param motor The ExtendedDcMotor used to power the drive train
     * @param gearTrain Array of Gear in the train in the order of motor, ..., wheel
     */
    public DriveTrain(Wheel wheel, RotationalDirection wheelRotatingForwardDirection, ExtendedDcMotor motor, Gear[] gearTrain){
        this.wheel = wheel;
        this.wheelRotatingForwardDirection = wheelRotatingForwardDirection;
        this.motor = motor;

        if (gearTrain == null){
            throw new IllegalArgumentException("gear train needed");
        }
        if (gearTrain.length<2){
            throw new IllegalArgumentException("not enough gears");
        }
        Gear motorGear = gearTrain[0];
        Gear wheelGear = gearTrain[gearTrain.length - 1];

        gearRatio = wheelGear.getNumTeeth() / motorGear.getNumTeeth();

        // adjust for motor vs. forward direction mismatch
        final boolean invertRotationalDirection;

        if (!motor.getMotorNativeDirection().equals(wheelRotatingForwardDirection)) {
            invertRotationalDirection = true;
        } else {
            invertRotationalDirection = false;
        }

        // Even number of gears does not reverse rotational direction
        if ( gearTrain.length % 2 == 0){
            direction = invertRotationalDirection ? 1 : -1;
        } else{
            direction = invertRotationalDirection ? -1 : 1;
        }

        if (direction == -1) {
            // Do this instead of inverting power ... it handles encoders for us
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public void driveInches(double linearInchesToDrive, double power) {
        int requiredEncoderCounts = getEncoderCountsForDriveInches(linearInchesToDrive);

        motor.setRelativeTargetPosition(requiredEncoderCounts);
        motor.setPower(power);
    }

    public void setRunMode(DcMotor.RunMode runMode) {
        motor.setMode(runMode);
    }

    private int getEncoderCountsForDriveInches(double linearInchesToDrive) {
        double wheelRevolutions = linearInchesToDrive / wheel.circumference;
        double motorRevolutions = wheelRevolutions * gearRatio;
        // this cast is safe will never have an encoder count that will reach beyond the max of an int
        int encoderCounts = (int)Math.round(motor.getEncoderCountsPerRevolution() * motorRevolutions);

        return encoderCounts;
    }
}
