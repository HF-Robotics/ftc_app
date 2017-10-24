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

package com.hfrobots.tnt.season1718;

import android.util.Log;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.bosch.NaiveAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public abstract class MecanumBotHardware extends OpMode {
    protected DcMotor[] motors;

    /**
     * Perform any actions that are necessary when the OpMode is enabled.
     * <p/>
     * The system calls this member once when the OpMode is enabled.
     */
    @Override
    public void init() {
        //
        // Use the hardwareMap to associate class members to hardware ports.
        //
        // Note that the names of the devices (i.e. arguments to the get method)
        // must match the names specified in the configuration file created by
        // the FTC Robot Controller (Settings-->Configure Robot).
        //
        // The variable below is used to provide telemetry data to a class user.
        //
        warningGenerated = false;
        warningMessage = "Can't map; ";

        try {
            leftFrontDriveMotor = hardwareMap.dcMotor.get("leftFrontDriveMotor");
            leftFrontDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        } catch (Exception ex) {
            appendWarningMessage("leftFrontDriveMotor");
            Log.e("tnt", ex.getLocalizedMessage());

            leftFrontDriveMotor = null;
        }

        try {
            leftRearDriveMotor = hardwareMap.dcMotor.get("leftRearDriveMotor");
            leftRearDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        } catch (Exception ex) {
            appendWarningMessage("leftRearDriveMotor");
            Log.e("tnt", ex.getLocalizedMessage());

            leftRearDriveMotor = null;
        }

        try {
            rightFrontDriveMotor = hardwareMap.dcMotor.get("rightFrontDriveMotor");
        } catch (Exception ex) {
            appendWarningMessage("rightFrontDriveMotor");
            Log.e("tnt", ex.getLocalizedMessage());

            rightFrontDriveMotor = null;
        }

        try {
            rightRearDriveMotor = hardwareMap.dcMotor.get("rightRearDriveMotor");
        } catch (Exception ex) {
            appendWarningMessage("rightRearDriveMotor");
            Log.e("tnt", ex.getLocalizedMessage());

            rightRearDriveMotor = null;
        }


        motors = new DcMotor[] { rightFrontDriveMotor, rightRearDriveMotor, leftFrontDriveMotor, leftRearDriveMotor};


        //try {
        //    leftColorRange = hardwareMap.get(LynxI2cColorRangeSensor.class, "leftColorRange");
        //} catch (Exception ex) {
        //    appendWarningMessage("leftColorRange");
        //    DbgLog.msg(ex.getLocalizedMessage());

        //    leftColorRange = null;
        //}

        //try {
        //    rightColorRange = hardwareMap.get(LynxI2cColorRangeSensor.class, "rightColorRange");
        //} catch (Exception ex) {
        //    appendWarningMessage("leftColorRange");
        //    DbgLog.msg(ex.getLocalizedMessage());

        //    rightColorRange = null;
        //}

        try {
            imu = hardwareMap.get(LynxEmbeddedIMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.mode                = BNO055IMU.SensorMode.IMU; // yes, it's the default, but let's be sure
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled      = false;
            parameters.loggingTag          = "IMU";
            //parameters.accelerationIntegrationAlgorithm = new NaiveAccelerationIntegrator();
            imu.initialize(parameters);
            imu.startAccelerationIntegration(null, null, 50); // not started by default?
        } catch (Exception ex) {
            appendWarningMessage("imu");
            Log.e("tnt", ex.getLocalizedMessage());

            imu = null;
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

    protected void stopAllDriveMotors() {
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

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
            // TODO Lauren implements this!

            // (1) Find the maximum magnitude of all wheel power values regardless of sign

            // (2) If that value is more than MAX_MOTOR_OUTPUT, scale it so the maximum magnitude
            //     is MAX_MOTOR_OUTPUT and *all other values* are scaled relative to that

            double maxMagnitude = 0;

            if (Math.abs(leftFront) > 1.0) {
                maxMagnitude = Math.abs(leftFront);
            }

            if (Math.abs(rightFront) > 1.0) {
                maxMagnitude = Math.abs(rightFront);
            }

            if (Math.abs(leftRear) > 1.0) {
                maxMagnitude = Math.abs(leftRear);
            }

            if (Math.abs(rightRear) > 1.0) {
                maxMagnitude = Math.abs(rightRear);
            }

            if (maxMagnitude > 1.0) {
                leftFront /= maxMagnitude;
                rightFront /= maxMagnitude;
                leftRear /= maxMagnitude;
                rightRear /= maxMagnitude;
            }
        }

        public String toString() {
            return "Wheel speed: " + leftFront + ", " + rightFront + ", " + leftRear + ", " + rightRear;
        }
    }

    private final static double MAX_MOTOR_OUTPUT = 1.0;

    public void mecanumDriveCartesian(double xPower, double yPower, double rotationPower, boolean inverted, double gyroAngle)
    {
        xPower = Range.clip(xPower, -1.0, 1.0);
        yPower = Range.clip(yPower, -1.0, 1.0);
        rotationPower = Range.clip(rotationPower, -1.0, 1.0);

        if (inverted) {
            xPower = -xPower;
            yPower = -yPower;
        }

        double cosA = Math.cos(Math.toRadians(gyroAngle));
        double sinA = Math.sin(Math.toRadians(gyroAngle));

        // Lauren - this formula should look familiar - do you recognize it?

        xPower = xPower * cosA - yPower * sinA;
        yPower = xPower * sinA + yPower * cosA;

        //if (gyroAssistEnabled)
        //{
        //    rotation += TrcUtil.clipRange(gyroAssistKp*(rotation - gyroRateScale*gyro.getZRotationRate().value));
        //}

        // leftFront, rightFront, leftRear, rightRear
        WheelSpeeds wheelSpeeds = new WheelSpeeds(xPower + yPower + rotationPower,
                                                  -xPower + yPower - rotationPower,
                                                  -xPower + yPower + rotationPower,
                                                  xPower + yPower - rotationPower);
        normalizeAndSetMotorPower(wheelSpeeds);
    }

    protected void normalizeAndSetMotorPower(WheelSpeeds wheelSpeeds) {
        wheelSpeeds.normalize();
        wheelSpeeds.clipToMaxOutput();

        if (leftFrontDriveMotor != null) {
            leftFrontDriveMotor.setPower(wheelSpeeds.leftFront);
        }
        if (rightFrontDriveMotor != null) {
            rightFrontDriveMotor.setPower(wheelSpeeds.rightFront);
        }
        if (leftRearDriveMotor != null) {
            leftRearDriveMotor.setPower(wheelSpeeds.leftRear);
        }
        if (rightRearDriveMotor != null) {
            rightRearDriveMotor.setPower(wheelSpeeds.rightRear);
        }

        Log.d("TNT", wheelSpeeds.toString());
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
    public void mecanumDrivePolar(double magnitude, double direction, double rotation, boolean inverted)
    {
        magnitude = Range.clip((magnitude * Math.sqrt(2.0)), -1.0, 1.0);

        if (inverted) {
            direction += 180.0;
            direction %= 360.0;
        }

        double dirInRad = Math.toRadians(direction + 45.0 /* Why +45 degrees? */);
        double cosD = Math.cos(dirInRad);
        double sinD = Math.sin(dirInRad);

        //if (gyroAssistEnabled)
        //{
        //    rotation += TrcUtil.clipRange(gyroAssistKp*(rotation - gyroRateScale*gyro.getZRotationRate().value));
        //}

        WheelSpeeds wheelSpeeds = new WheelSpeeds(sinD * magnitude + rotation,
                                                  cosD * magnitude - rotation,
                                                  cosD * magnitude + rotation,
                                                  sinD * magnitude - rotation);
        normalizeAndSetMotorPower(wheelSpeeds);
    }

    /**
     * Indicate whether a message is a available to the class user.
     */
    private boolean warningGenerated = false;

    /**
     * Store a message to the user if one has been generated.
     */
    private String warningMessage;

    protected DcMotor leftFrontDriveMotor;

    protected DcMotor rightFrontDriveMotor;

    protected DcMotor leftRearDriveMotor;

    protected DcMotor rightRearDriveMotor;

    protected LynxEmbeddedIMU imu;

    // protected LynxI2cColorRangeSensor leftColorRange;

    // protected LynxI2cColorRangeSensor rightColorRange;
}
