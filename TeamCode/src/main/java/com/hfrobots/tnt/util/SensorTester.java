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

package com.hfrobots.tnt.util;

import android.text.method.Touch;

import com.hfrobots.tnt.corelib.control.DebouncedButton;
import com.hfrobots.tnt.corelib.control.NinjaGamePad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

/**
 * An OpMode that allows you to test any/all of the sensors on a robot
 *
 * gamepad1.left_bumper = cycle through sensor types
 * gamepad1.right_bumper = cycle through all configured sensors of current type
 *
 * current sensor type, name and data are displayed in driver station telemetry
 *
 */
@TeleOp(name="Sensor Tester", group="Utilities")
@SuppressWarnings("unused")
public class SensorTester extends OpMode {
    private List<NamedDeviceMap.NamedDevice<ColorSensor>> namedColorSensors;

    private List<NamedDeviceMap.NamedDevice<I2cDevice>> namedI2cDevices;

    private List<NamedDeviceMap.NamedDevice<GyroSensor>> namedGyroDevices;

    private List<NamedDeviceMap.NamedDevice<CompassSensor>> namedCompassDevices;

    private List<NamedDeviceMap.NamedDevice<TouchSensor>> namedTouchDevices;


    private int currentListPosition;

    private DebouncedButton leftBumper;
    private DebouncedButton rightBumper;

    private DebouncedButton dpadUp;

    private DebouncedButton dpadDown;

    private DebouncedButton aButton;

    private NamedDeviceMap namedDeviceMap;

    @Override
    public void init() {
        namedDeviceMap = new NamedDeviceMap(hardwareMap);

        namedColorSensors = namedDeviceMap.getAll(ColorSensor.class);
        namedI2cDevices = namedDeviceMap.getAll(I2cDevice.class);
        namedGyroDevices = namedDeviceMap.getAll(GyroSensor.class);
        namedCompassDevices = namedDeviceMap.getAll(CompassSensor.class);
        namedTouchDevices = namedDeviceMap.getAll(TouchSensor.class);

        for (NamedDeviceMap.NamedDevice<GyroSensor> namedGyro : namedGyroDevices) {
            namedGyro.getDevice().calibrate();
        }

        currentListPosition = 0;

        NinjaGamePad ninjaGamePad = new NinjaGamePad(gamepad1);
        leftBumper = new DebouncedButton(ninjaGamePad.getLeftBumper());
        rightBumper = new DebouncedButton(ninjaGamePad.getRightBumper());
        dpadUp = new DebouncedButton(ninjaGamePad.getDpadUp());
        dpadDown = new DebouncedButton(ninjaGamePad.getDpadDown());
        aButton = new DebouncedButton(ninjaGamePad.getAButton());
    }

    private boolean ledEnabled = false;

    private enum Mode { COLOR, RANGE, GYRO, COMPASS, TOUCH }

    private Mode currentMode = Mode.COLOR;

    @Override
    public void loop() {
        if (leftBumper.getRise()) {
            switch (currentMode) {
                case COLOR:
                    currentMode = Mode.RANGE;
                    break;
                case RANGE:
                    currentMode = Mode.GYRO;
                    break;
                case GYRO:
                    currentMode = Mode.COMPASS;
                    break;
                case COMPASS:
                    currentMode = Mode.TOUCH;
                    break;
                case TOUCH:
                    currentMode = Mode.COLOR;
                    break;
            }
        }

        switch (currentMode) {
            case COLOR:
                doColorSensorLoop();
                break;
            case RANGE:
                doRangeSensorLoop();
                break;
            case GYRO:
                doGyroSensorLoop();
                break;
            case COMPASS:
                doCompassSensorLoop();
                break;
            case TOUCH:
                doTouchSensorLoop();
                break;
        }
    }

    private int rangeSensorListPosition = 0;

    private I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    private static final int RANGE1_REG_START = 0x04; //Register to start reading
    private static final int RANGE1_READ_LENGTH = 2; //Number of byte to read

    private void doRangeSensorLoop() {
        if (namedI2cDevices.isEmpty()) {
            telemetry.addData("No range sensors", "");
            updateTelemetry(telemetry);
            return;
        }

        if (rightBumper.getRise()) {
            rangeSensorListPosition++;

            if (rangeSensorListPosition == namedI2cDevices.size()) {
                rangeSensorListPosition = 0;
            }
        }

        NamedDeviceMap.NamedDevice<I2cDevice> currentI2cDevice = namedI2cDevices.get(rangeSensorListPosition);

        if (!currentI2cDevice.getName().contains("range")) {
            telemetry.addData("Not a range sensor: ", "%s", currentI2cDevice.getName());
            updateTelemetry(telemetry);
            return;
        }

        I2cDevice rangeSensorRaw = currentI2cDevice.getDevice();
        I2cDeviceSynch RANGE1Reader = new I2cDeviceSynchImpl(rangeSensorRaw, RANGE1ADDRESS, false);
        RANGE1Reader.engage();

        byte[] range1Cache; //The read will return an array of bytes.
        range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);

        int ultrasonicRange = range1Cache[0] & 0xFF;
        int opticalDistance = range1Cache[1] & 0xFF;

        telemetry.addData("range ",  "%s", currentI2cDevice.getName());
        telemetry.addData("Ultrasonic", "%d", ultrasonicRange);
        telemetry.addData("Optical", "%d", opticalDistance);
        updateTelemetry(telemetry);
    }

    private void doColorSensorLoop() {
        namedColorSensors = namedDeviceMap.getAll(ColorSensor.class);

        if (namedColorSensors.isEmpty()) {
            telemetry.addData("No color sensors", "");
            updateTelemetry(telemetry);
            return;
        }

        if (rightBumper.getRise()) {
            currentListPosition++;

            if (currentListPosition == namedColorSensors.size()) {
                currentListPosition = 0;
            }
        }

        NamedDeviceMap.NamedDevice<ColorSensor> currentNamedColorSensor = namedColorSensors.get(currentListPosition);
        ColorSensor currentColorSensor = currentNamedColorSensor.getDevice();
        String sensorName = currentNamedColorSensor.getName();

        if (aButton.getRise()) {
            ledEnabled = !ledEnabled;
        }

        currentColorSensor.enableLed(ledEnabled);
        int alpha = currentColorSensor.alpha();
        int red = currentColorSensor.red();
        int green = currentColorSensor.green();
        int blue = currentColorSensor.blue();

        telemetry.addData("color ",  "%s", sensorName);
        telemetry.addData("R", "%d", red);
        telemetry.addData("G", "%d", green);
        telemetry.addData("B", "%d", blue);
        telemetry.addData("A", "%d", alpha);
        updateTelemetry(telemetry);
    }

    private int currentGyroListPosition = 0;

    private void doGyroSensorLoop() {
        if (namedGyroDevices.isEmpty()) {
            telemetry.addData("No gyro sensors", "");
            updateTelemetry(telemetry);
            return;
        }

        if (rightBumper.getRise()) {
            currentGyroListPosition++;

            if (currentGyroListPosition == namedGyroDevices.size()) {
                currentGyroListPosition = 0;
            }
        }

        NamedDeviceMap.NamedDevice<GyroSensor> currentNamedGyroSensor = namedGyroDevices.get(currentGyroListPosition);
        GyroSensor currentGyroSensor = currentNamedGyroSensor.getDevice();
        String sensorName = currentNamedGyroSensor.getName();

        int rawX = currentGyroSensor.rawX();
        int rawY = currentGyroSensor.rawY();
        int rawZ = currentGyroSensor.rawZ();
        int heading = currentGyroSensor.getHeading();

        telemetry.addData("gyro ",  "%s", sensorName);
        telemetry.addData("X", "%d", rawX);
        telemetry.addData("Y", "%d", rawY);
        telemetry.addData("Z", "%d", rawZ);
        telemetry.addData("H", "%d", heading);
        updateTelemetry(telemetry);
    }

    private int currentCompassListPosition = 0;

    private void doCompassSensorLoop() {
        if (namedCompassDevices.isEmpty()) {
            telemetry.addData("No compass sensors", "");
            updateTelemetry(telemetry);
            return;
        }

        if (rightBumper.getRise()) {
            currentCompassListPosition++;

            if (currentCompassListPosition == namedCompassDevices.size()) {
                currentCompassListPosition = 0;
            }
        }

        NamedDeviceMap.NamedDevice<CompassSensor> currentNamedCompassSensor = namedCompassDevices.get(currentCompassListPosition);
        CompassSensor currentCompassSensor = currentNamedCompassSensor.getDevice();
        currentCompassSensor.setMode(CompassSensor.CompassMode.MEASUREMENT_MODE);
        String sensorName = currentNamedCompassSensor.getName();

        double direction = currentCompassSensor.getDirection();
        telemetry.addData("compass ",  "%s", sensorName);
        telemetry.addData("direction", "%s", Double.toString(direction));
        updateTelemetry(telemetry);
    }

    private int currentTouchSensorListPosition = 0;

    private void doTouchSensorLoop() {
        if (namedTouchDevices.isEmpty()) {
            telemetry.addData("No touch sensors", "");
            updateTelemetry(telemetry);
            return;
        }

        if (rightBumper.getRise()) {
            currentTouchSensorListPosition++;

            if (currentTouchSensorListPosition == namedCompassDevices.size()) {
                currentTouchSensorListPosition = 0;
            }
        }

        NamedDeviceMap.NamedDevice<TouchSensor> currentNamedTouchSensor = namedTouchDevices.get(currentTouchSensorListPosition);
        TouchSensor currentTouchSensor = currentNamedTouchSensor.getDevice();
        String sensorName = currentNamedTouchSensor.getName();
        boolean isPressed = currentTouchSensor.isPressed();
        telemetry.addData("touch ",  "%s", sensorName);
        telemetry.addData("is pressed", "%s", Boolean.toString(isPressed));
        updateTelemetry(telemetry);
    }
}