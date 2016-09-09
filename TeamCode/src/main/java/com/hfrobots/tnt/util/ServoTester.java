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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import com.hfrobots.tnt.corelib.control.DebouncedButton;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.Callable;

/**
 * An OpMode that allows you to test any/all of the servos on a robot
 *
 * gamepad1.left_stick.y = proportional adjustment
 * gamepad1.dpad up/down = micro adjustment
 * gamepad1.left_bumper = cycle through all configured servos
 *
 * current servo name and position are displayed in driver station telemetry
 *
 */
@TeleOp(name="Servo Tester", group="Utilities")
public class ServoTester extends OpMode {
    private List<Servo> servos;
    private Map<Servo, String> servosToNames = new HashMap<>();
    private int currentListPosition;

    private DebouncedButton rightBumper = new DebouncedButton(new Callable<Boolean>() {
        @Override
        public Boolean call() throws Exception {
            return gamepad1.right_bumper;
        }
    }, 100);

    private DebouncedButton dpadUp = new DebouncedButton(new Callable<Boolean>() {
        @Override
        public Boolean call() throws Exception {
            return gamepad1.dpad_up;
        }
    }, 100);

    private DebouncedButton dpadDown = new DebouncedButton(new Callable<Boolean>() {
        @Override
        public Boolean call() throws Exception {
            return gamepad1.dpad_down;
        }
    }, 100);

    @Override
    public void init() {
        // Build a map of servo -> name, since FTC doesn't give us one
        // and servos can't tell us their name either (boo, hiss)
        for (Map.Entry<String, Servo> entry : hardwareMap.servo.entrySet()) {
            servosToNames.put(entry.getValue(), entry.getKey());
        }

        servos = hardwareMap.getAll(Servo.class);
        currentListPosition = 0;

        for (Servo servo : servos) {
            servo.setPosition(0);
        }
    }

    @Override
    public void loop() {
        if (servos.isEmpty()) {
            telemetry.addData("No servos", "");
            updateTelemetry(telemetry);
            return;
        }

        if (rightBumper.getDebouncedPressed()) {
            currentListPosition++;

            if (currentListPosition == servos.size()) {
                currentListPosition = 0;
            }

        }

        Servo currentServo = servos.get(currentListPosition);
        String servoName = servosToNames.get(currentServo);
        double currentPosition = currentServo.getPosition();

        float leftStickYPosition = -gamepad1.left_stick_y * .1f;
        double microAdjust;

        if (dpadUp.getDebouncedPressed()) {
            microAdjust = 0.001;
        } else if (dpadDown.getDebouncedPressed()) {
            microAdjust = -0.001;
        } else {
            microAdjust = 0;
        }

        double newPosition = currentPosition + leftStickYPosition + microAdjust;
        newPosition = Range.clip(newPosition, 0, 1.0);
        telemetry.addData("servo ",  "%s - %s", servoName, Double.toString(newPosition));
        updateTelemetry(telemetry);

        currentServo.setPosition(newPosition);
    }
}
