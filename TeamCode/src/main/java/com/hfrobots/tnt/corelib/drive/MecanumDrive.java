/**
 Copyright (c) 2017 HF Robotics (http://www.hfrobots.com)
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

import com.qualcomm.robotcore.util.Range;

public class MecanumDrive {
    private final static double MAX_MOTOR_OUTPUT = 1.0;

    // Our experimental chassis didn't do autonomous, so had motors ... what lets us
    // do things like drive inches - given that our competition drive base uses sprockets?
    // FIXME: Add the required objects here to control the motors and wheels for driving

    // This is from our experimental chassis from the off-season, now we'll reuse it
    protected static class WheelSpeeds {
        protected double leftFront;
        protected double rightFront;
        protected double leftRear;
        protected double rightRear;

        WheelSpeeds(double leftFront, double rightFront, double leftRear, double rightRear) {
            this.leftFront = leftFront;
            this.rightFront = rightFront;
            this.leftRear = leftRear;
            this.rightRear = rightRear;
        }

        void clipToMaxOutput() {
            leftFront = Range.clip(leftFront, -MAX_MOTOR_OUTPUT, MAX_MOTOR_OUTPUT);
            rightFront = Range.clip(rightFront, -MAX_MOTOR_OUTPUT, MAX_MOTOR_OUTPUT);
            leftRear = Range.clip(leftRear, -MAX_MOTOR_OUTPUT, MAX_MOTOR_OUTPUT);
            rightRear = Range.clip(rightRear, -MAX_MOTOR_OUTPUT, MAX_MOTOR_OUTPUT);
        }

        void normalize() {
            // (1) Find the maximum magnitude of all wheel power values regardless of sign

            // (2) If that value is more than MAX_MOTOR_OUTPUT, scale it so the maximum magnitude
            //     is MAX_MOTOR_OUTPUT and *all other values* are scaled relative to that

            double maxMagnitude = 0;

            if (Math.abs(leftFront) > MAX_MOTOR_OUTPUT) {
                maxMagnitude = Math.abs(leftFront);
            }

            if (Math.abs(rightFront) > MAX_MOTOR_OUTPUT) {
                maxMagnitude = Math.abs(rightFront);
            }

            if (Math.abs(leftRear) > MAX_MOTOR_OUTPUT) {
                maxMagnitude = Math.abs(leftRear);
            }

            if (Math.abs(rightRear) > MAX_MOTOR_OUTPUT) {
                maxMagnitude = Math.abs(rightRear);
            }

            if (maxMagnitude > MAX_MOTOR_OUTPUT) {
                leftFront /= maxMagnitude;
                rightFront /= maxMagnitude;
                leftRear /= maxMagnitude;
                rightRear /= maxMagnitude;
            }
        }
    }

    /**
     * Return a factory "builder" for this class ... it's too easy to mix up which
     * arguments to the constructor are used in what order and that will cause malfunctions
     *
     * The "Builder Pattern" solves that problem
     */
    public static Builder builder() {
        return new Builder();
    }

    public static class Builder {
        private Builder() {} // shouldn't call directly

        public void addLeftFrontDrive(Object obj) {

        }

        public void addRightFrontDrive(Object obj) {

        }

        public void addLeftRearDrive(Object obj) {

        }

        public void addRightRearDrive(Object obj) {

        }

        public MecanumDrive build() {
            if (false) { // FIXME: What would mean we can't build the drive?
                throw new IllegalArgumentException("");
            }

            return new MecanumDrive();
        }
    }

    // FIXME: We need to add access to the "motors" somehow
    // They also need to be configured so that "forward" spins them all in the correct direction
    private MecanumDrive() {
    }

    /**
     * Drives the given number of inches in the Y axis direction using the given power level
     * @param inches
     * @param power
     */
    public void driveYAxisInches(double inches, double power) {
        // FIXME
        throw new RuntimeException("Not implemented");
    }

    /**
     * Drives the given number of inches in the X axis direction using the given power level
     * @param inches
     * @param power
     */
    public void driveXAxisInches(double inches, double power) {
        // FIXME
        throw new RuntimeException("Not implemented");
    }

    // FIXME: When we're done driving inches along an axis ... we need to reset to not use encoders
    // to direct control (or do it before Teleop starts as well)...what exactly should we do
    // and should we ever do it automatically?
    public void resetToDirectControl() {
        throw new RuntimeException("Not implemented");
    }

    public void driveCartesian(double xPower, double yPower, double rotationPower, boolean inverted, double gyroAngle) {
        xPower = Range.clip(xPower, -1.0, 1.0);
        yPower = Range.clip(yPower, -1.0, 1.0);
        rotationPower = Range.clip(rotationPower, -1.0, 1.0);

        if (inverted) {
            xPower = -xPower;
            yPower = -yPower;
        }

        double cosA = Math.cos(Math.toRadians(gyroAngle));
        double sinA = Math.sin(Math.toRadians(gyroAngle));

        // This is 2D matrix rotation transform - like we did with Skittlebot in season 1

        xPower = xPower * cosA - yPower * sinA;
        yPower = xPower * sinA + yPower * cosA;

        WheelSpeeds wheelSpeeds = new WheelSpeeds(
                 xPower + yPower + rotationPower,
                -xPower + yPower - rotationPower,
                -xPower + yPower + rotationPower,
                 xPower + yPower - rotationPower);

        normalizeAndSetMotorPower(wheelSpeeds);
    }

    /**
     * Are any motors powering this drive busy with PID control?
     */
    public boolean isBusy() {
        // FIXME
        throw new RuntimeException("Not implemented");
    }

    protected void normalizeAndSetMotorPower(WheelSpeeds wheelSpeeds) {
        wheelSpeeds.normalize();
        wheelSpeeds.clipToMaxOutput();

        // FIXME: Need to send power commands to motors
        throw new RuntimeException("Not implemented");
    }

    /**
     * This method implements mecanum drive where magnitude controls how fast the robot will go in the given direction
     * and how fast it will rotate.
     *
     * @param magnitude specifies the magnitude combining x and y axes.
     * @param direction specifies the direction in degrees.
     * @param rotation specifies the rotation power.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    // currently not used, but interesting to make work later
//    public void drivePolar(double magnitude, double direction, double rotation, boolean inverted)
//    {
//        magnitude = Range.clip((magnitude * Math.sqrt(2.0)), -1.0, 1.0);
//
//        if (inverted) {
//            direction += 180.0;
//            direction %= 360.0;
//        }
//
//        double dirInRad = Math.toRadians(direction + 45.0 /* Why +45 degrees? */);
//        double cosD = Math.cos(dirInRad);
//        double sinD = Math.sin(dirInRad);
//
//        WheelSpeeds wheelSpeeds = new WheelSpeeds(sinD * magnitude + rotation,
//                cosD * magnitude - rotation,
//                cosD * magnitude + rotation,
//                sinD * magnitude - rotation);
//        normalizeAndSetMotorPower(wheelSpeeds);
//    }
}
