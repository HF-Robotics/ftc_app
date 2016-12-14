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

import com.hfrobots.tnt.corelib.control.DebouncedButton;
import com.hfrobots.tnt.corelib.control.NinjaGamePad;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * An OpMode that allows you to test any/all of the sensors on a robot
 *
 * gamepad1.left_bumper = cycle through sensor types
 * gamepad1.right_bumper = cycle through all configured sensors of current type
 *
 * current sensor type, name and data are displayed in driver station telemetry
 *
 */
@TeleOp(name="Range Sensor Tester", group="Utilities")
@SuppressWarnings("unused")
public class RangeSensorTester extends OpMode {
    private ModernRoboticsI2cRangeSensor rangeSensor;

    @Override
    public void init() {
        NinjaGamePad ninjaGamePad = new NinjaGamePad(gamepad1);
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
    }

    private boolean ledEnabled = false;

    @Override
    public void loop() {

        double cmOptical = rangeSensor.cmOptical();
        double cmUltrasonic = rangeSensor.cmUltrasonic();
        double cmDistance = rangeSensor.getDistance(DistanceUnit.CM);

        telemetry.addData("Range ",  "%s", "range");
        telemetry.addData("optical", "%5.2f cm", cmOptical);
        telemetry.addData("ultrasonic", "%5.2f cm", cmUltrasonic);
        telemetry.addData("auto", "%5.2f cm", cmDistance);
        updateTelemetry(telemetry);
    }
}
