/**
 Copyright (c) 2016, 2017, HF Robotics (http://www.hfrobots.com)
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

import com.hfrobots.tnt.corelib.control.DebouncedButton;
import com.hfrobots.tnt.corelib.control.NinjaGamePad;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
    private List<NamedDeviceMap.NamedDevice<ModernRoboticsI2cColorSensor>> namedColorSensors;

    private List<NamedDeviceMap.NamedDevice<ModernRoboticsI2cRangeSensor>> namedRangeSensors;

    private List<NamedDeviceMap.NamedDevice<OpticalDistanceSensor>> namedOdsSensors;

    private List<NamedDeviceMap.NamedDevice<GyroSensor>> namedGyroDevices;

    private List<NamedDeviceMap.NamedDevice<CompassSensor>> namedCompassDevices;

    private List<NamedDeviceMap.NamedDevice<TouchSensor>> namedTouchDevices;

    private List<NamedDeviceMap.NamedDevice<LynxI2cColorRangeSensor>> namedLynxColorSensors;

    private List<NamedDeviceMap.NamedDevice<DigitalChannel>> namedDigitalChannels;

    private int currentListPosition;

    private DebouncedButton leftBumper;
    private DebouncedButton rightBumper;

    private DebouncedButton dpadUp;

    private DebouncedButton dpadDown;

    private DebouncedButton aButton;

    private DebouncedButton xButton;

    private DebouncedButton yButton;

    private NamedDeviceMap namedDeviceMap;

    @Override
    public void init() {
        namedDeviceMap = new NamedDeviceMap(hardwareMap);

        namedDigitalChannels = namedDeviceMap.getAll(DigitalChannel.class);
        namedLynxColorSensors = namedDeviceMap.getAll(LynxI2cColorRangeSensor.class);
        namedColorSensors = namedDeviceMap.getAll(ModernRoboticsI2cColorSensor.class);
        namedRangeSensors = namedDeviceMap.getAll(ModernRoboticsI2cRangeSensor.class);
        namedOdsSensors = namedDeviceMap.getAll(OpticalDistanceSensor.class);
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
        xButton = new DebouncedButton(ninjaGamePad.getXButton());
        yButton = new DebouncedButton(ninjaGamePad.getYButton());
    }

    private boolean ledEnabled = false;

    private enum Mode { COLOR, LYNX_COLOR, RANGE, ODS, DIGITAL_CHANNEL, GYRO, COMPASS, TOUCH }

    private Mode currentMode = Mode.COLOR;

    @Override
    public void loop() {
        if (leftBumper.getRise()) {
            switch (currentMode) {
                case COLOR:
                    currentMode = Mode.LYNX_COLOR;
                    break;
                case LYNX_COLOR:
                    currentMode = Mode.RANGE;
                    break;
                case RANGE:
                    currentMode = Mode.ODS;
                    break;
                case ODS:
                    currentMode = Mode.DIGITAL_CHANNEL;
                    break;
                case DIGITAL_CHANNEL:
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
            case LYNX_COLOR:
                doLynxColorSensorLoop();
                break;
            case RANGE:
                doRangeSensorLoop();
                break;
            case ODS:
                doOdsSensorLoop();
                break;
            case DIGITAL_CHANNEL:
                doDigitalChannelLoop();
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

    private int currentOpticalDistanceSensorListPosition = 0;

    private void doOdsSensorLoop() {
        namedOdsSensors = namedDeviceMap.getAll(OpticalDistanceSensor.class);


        if (namedOdsSensors.isEmpty()) {
            telemetry.addData("No optical distance sensors", "");
            updateTelemetry(telemetry);
            return;
        }

        if (rightBumper.getRise()) {
            currentOpticalDistanceSensorListPosition++;

            if (currentOpticalDistanceSensorListPosition == namedOdsSensors.size()) {
                currentOpticalDistanceSensorListPosition = 0;
            }
        }

        NamedDeviceMap.NamedDevice<OpticalDistanceSensor> currentNamedOdsSensor = namedOdsSensors.get(currentOpticalDistanceSensorListPosition);
        OpticalDistanceSensor currentOdsSensor = currentNamedOdsSensor.getDevice();
        String sensorName = currentNamedOdsSensor.getName();
        telemetry.addData("ODS ",  "%s", sensorName);
        telemetry.addData("Raw cm", "%f", currentOdsSensor.getRawLightDetected());
        telemetry.addData("Calibrated cm", "%f", currentOdsSensor.getLightDetected());
        updateTelemetry(telemetry);
    }

    private int currentRangeSensorListPosition = 0;

    private void doRangeSensorLoop() {
        namedRangeSensors = namedDeviceMap.getAll(ModernRoboticsI2cRangeSensor.class);

        if (namedRangeSensors.isEmpty()) {
            telemetry.addData("No range sensors", "");
            updateTelemetry(telemetry);
            return;
        }

        if (rightBumper.getRise()) {
            currentRangeSensorListPosition++;

            if (currentRangeSensorListPosition == namedRangeSensors.size()) {
                currentRangeSensorListPosition = 0;
            }
        }

        NamedDeviceMap.NamedDevice<ModernRoboticsI2cRangeSensor> currentNamedRangeSensor = namedRangeSensors.get(currentRangeSensorListPosition);
        ModernRoboticsI2cRangeSensor currentRangeSensor = currentNamedRangeSensor.getDevice();
        String sensorName = currentNamedRangeSensor.getName();
        telemetry.addData("range ",  "%s", sensorName);
        telemetry.addData("Ultrasonic cm", "%f", currentRangeSensor.cmUltrasonic());
        telemetry.addData("Optical cm", "%f", currentRangeSensor.cmOptical());
        updateTelemetry(telemetry);
    }

    private void doColorSensorLoop() {
        namedColorSensors = namedDeviceMap.getAll(ModernRoboticsI2cColorSensor.class);

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

        NamedDeviceMap.NamedDevice<ModernRoboticsI2cColorSensor> currentNamedColorSensor = namedColorSensors.get(currentListPosition);
        ModernRoboticsI2cColorSensor currentColorSensor = currentNamedColorSensor.getDevice();
        String sensorName = currentNamedColorSensor.getName();

        if (aButton.getRise()) {
            ledEnabled = !ledEnabled;
        }

        currentColorSensor.enableLed(ledEnabled);
        int alpha = currentColorSensor.alpha();
        int red = currentColorSensor.red();
        int green = currentColorSensor.green();
        int blue = currentColorSensor.blue();

        telemetry.addData("color ",  "%s x blk cal, y wt cal", sensorName);
        telemetry.addData("R", "%d", red);
        telemetry.addData("G", "%d", green);
        telemetry.addData("B", "%d", blue);
        telemetry.addData("A", "%d", alpha);
        updateTelemetry(telemetry);
    }

    int currentLynxColorListPosition = 0;

    private void doLynxColorSensorLoop() {
        namedLynxColorSensors = namedDeviceMap.getAll(LynxI2cColorRangeSensor.class);

        if (namedLynxColorSensors.isEmpty()) {
            telemetry.addData("No lynx i2c color sensors", "");
            updateTelemetry(telemetry);
            return;
        }

        if (rightBumper.getRise()) {
            currentLynxColorListPosition++;

            if (currentLynxColorListPosition == namedLynxColorSensors.size()) {
                currentLynxColorListPosition = 0;
            }
        }

        NamedDeviceMap.NamedDevice<LynxI2cColorRangeSensor> currentNamedLynxColorSensor = namedLynxColorSensors.get(currentLynxColorListPosition);
        LynxI2cColorRangeSensor currentColorSensor = currentNamedLynxColorSensor.getDevice();
        String sensorName = currentNamedLynxColorSensor.getName();

        if (aButton.getRise()) {
            ledEnabled = !ledEnabled;
        }

        currentColorSensor.enableLed(ledEnabled);
        int alpha = currentColorSensor.alpha();
        int red = currentColorSensor.red();
        int green = currentColorSensor.green();
        int blue = currentColorSensor.blue();

        telemetry.addData("color ",  "%s x blk cal, y wt cal", sensorName);
        telemetry.addData("R G B A", "%d %d %d %d", red, green, blue, alpha);
        telemetry.addData("D (mm):", "%f", currentColorSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("light:", "%f", currentColorSensor.getLightDetected());
        telemetry.addData("raw light:", "%f", currentColorSensor.getRawLightDetected());
        updateTelemetry(telemetry);
    }

    int currentDigitalChannelListPosition = 0;

    private void doDigitalChannelLoop() {
        namedLynxColorSensors = namedDeviceMap.getAll(LynxI2cColorRangeSensor.class);

        if (namedDigitalChannels.isEmpty()) {
            telemetry.addData("No digital channels", "");
            updateTelemetry(telemetry);
            return;
        }

        if (rightBumper.getRise()) {
            currentDigitalChannelListPosition++;

            if (currentDigitalChannelListPosition == namedDigitalChannels.size()) {
                currentDigitalChannelListPosition = 0;
            }
        }

        NamedDeviceMap.NamedDevice<DigitalChannel> currentNamedDigitalChannel = namedDigitalChannels.get(currentDigitalChannelListPosition);
        DigitalChannel currentDigitalChannel = currentNamedDigitalChannel.getDevice();
        currentDigitalChannel.setMode(DigitalChannel.Mode.INPUT);
        String sensorName = currentNamedDigitalChannel.getName();

        telemetry.addData("digital ",  "%s = %s", sensorName, String.valueOf(currentDigitalChannel.getState()));

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
