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

package com.hfrobots.tnt.corelib;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveTrain {
    public enum RotationDirection { CLOCKWISE, COUNTERCLOCKWISE };

    private final Wheel wheel;
    private final RotationDirection rotationForward;
    private final NeverestMotor motor;
    private final double gearRatio;
    private final int gearDirectionMultiplier;

    /**
     * Stupid silly example
     */

    public void exampleUse() {
        Wheel andyMarkWheel = new Wheel(4.0);
        NeverestMotor motor = new NeverestMotor(null /* in real life, get from hw map */);
        Gear smallGear = new Gear(16);
        Gear largeGear = new Gear(48);
        DriveTrain rightSideDrivenWheel = new DriveTrain(motor, andyMarkWheel,
                RotationDirection.CLOCKWISE, smallGear, largeGear);

        // Example to just test that the math works, without actually using any real motor
        System.out.println("Pulses needed to drive 8 inches on a 4 inch wheel with given gear train: "
                + rightSideDrivenWheel.getPulsesForDistanceInches(8));
        System.out.println("Direction motor needs to turn: "
                + rightSideDrivenWheel.getDirectionForDistanceInches(8));

        // Or, alternatively, use the real motor
        rightSideDrivenWheel.driveDistanceInches(8, 1.0);

        // In a state, of a state machine, we would the loop on this if statement
        // until it returns false

        while (motor.isBusy()) {
            // keep waiting
        }
    }

    /**
     * Creates a new DriveTrain with the given motor, wheel, direction of rotation that moves
     * drivetrain forward, and a list of gears comprising the gear train, in order
     * from the motor to the wheel.
     */
    public DriveTrain(NeverestMotor m, Wheel w, RotationDirection forward, Gear... gears) {
        wheel = w;
        rotationForward = forward;
        motor = m;

        if (gears == null || gears.length == 0) {
            gearRatio = 1; // why?
            gearDirectionMultiplier = 1; // why?
        } else {
            // The syntax Gear... is shorthand for "Take any number of Gear instances
            // give them to me as an array (which also includes none)

            Gear firstGear = gears[0];
            Gear lastGear = gears[gears.length - 1];

            // Let the kids come to this, why is the gear ratio computed in the direction
            // of gear driving wheel to driven gear?
            gearRatio = lastGear.getNumberOfTeeth() / firstGear.getNumberOfTeeth();

            if (gears.length % 2 == 0) {
                gearDirectionMultiplier = -1;
            } else {
                gearDirectionMultiplier = 1;
            }
        }
    }

    /**
     * Computes the number of encoder pulses needed to have the motor driving
     * the gear train and thus the wheel rotate the required amount of times
     * to end up driving the wheel the distance given in the variable distanceInches
     */
    public int getPulsesForDistanceInches(double distanceInches) {
        double requiredWheelRotations = wheel.getRevolutionsForLinearTravel(distanceInches);
        double requiredMotorRotations = requiredWheelRotations * gearRatio;
        int requiredEncoderPulses = (int)Math.round(motor.getPulsesPerRevolution() * requiredMotorRotations);

        return requiredEncoderPulses;
    }

    /**
     * Given the distance (positive forward, negative reverse), compute the DcMotor
     * direction needed, taking into consideration the rotation direction for
     * the driven wheel that represents "forward"
     */
    public DcMotor.Direction getDirectionForDistanceInches(double distanceInches) {
        final DcMotor.Direction requiredDirection;

        switch (rotationForward) {
            case COUNTERCLOCKWISE:
                if (distanceInches < 0) {
                    requiredDirection = DcMotor.Direction.REVERSE;
                } else {
                    requiredDirection = DcMotor.Direction.FORWARD;
                }
                break;
            case CLOCKWISE:
                if (distanceInches < 0) {
                    requiredDirection = DcMotor.Direction.FORWARD;
                } else {
                    requiredDirection = DcMotor.Direction.REVERSE;
                }
                break;
            default:
                throw new IllegalArgumentException();
        }

        return requiredDirection;
    }

    public void driveDistanceInches(double distanceInches, double power) {
        int requiredEncoderPulses = getPulsesForDistanceInches(distanceInches);

        DcMotor.Direction requiredDirection = getDirectionForDistanceInches(distanceInches);

        motor.runToPositionWithPower(requiredEncoderPulses, power, requiredDirection);
    }

    static class Wheel {
        private final double circumferenceInches;

        public Wheel(double diameterInches) {
            circumferenceInches = Math.PI * diameterInches; // of course, have team derive this on their own!
        }

        public double getRevolutionsForLinearTravel(double distanceInches) {
            if (distanceInches == 0) {
                return 0; // avoid divide by zero error
            }

            return distanceInches / circumferenceInches;
        }
    }

    static class Gear {
        private final int numberOfTeeth;

        public Gear(int teeth) {
            numberOfTeeth = teeth;
        }

        public int getNumberOfTeeth() {
            return numberOfTeeth;
        }
    }

    static class NeverestMotor {
        private final DcMotor actualMotor;

        public NeverestMotor(DcMotor motor) {
            actualMotor = motor;
        }

        public int getPulsesPerRevolution() {
            return 140; // from AndyMark documentation
        }

        public void resetEncoders() {
            actualMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        }

        public void runToPositionWithPower(int encoderTicks, double power, DcMotor.Direction direction) {
            // these are just guesses (in what order is needed)
            // we either need to research it, or experiment
            // the documentation with the SDK says zip, zilch, nada
            // about what order things need called in, or what they actually do
            // (it is in fact, a horrible example for the students)
            actualMotor.setDirection(direction);
            // tell the motors where we're going (don't need to reset encoders unless needed
            // since basing this on the current position makes it all relative
            actualMotor.setTargetPosition(actualMotor.getCurrentPosition() + encoderTicks);
            actualMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            actualMotor.setPower(power);

            // Note, any state, opMode using this will now need to loop until !isBusy
        }

        public boolean isBusy() {
            return actualMotor.isBusy();
        }
    }
}
