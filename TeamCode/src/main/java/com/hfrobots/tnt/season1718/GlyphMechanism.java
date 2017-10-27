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

package com.hfrobots.tnt.season1718;

import android.util.Log;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

public class GlyphMechanism {
    private Servo upperGripper;
    private Servo lowerGripper;
    private final CRServo rotate;
    private final Servo gripper1;
    private final Servo gripper2;
    private boolean isFlipped;
    private final DigitalChannel ccwLimitSwitch;
    private final DigitalChannel cwLimitSwitch;

    private static final double GRIPPER_OPEN = 0.5;
    private static final double GRIPPER_CLOSED = 0;


    public GlyphMechanism(Servo naturalTopGripper, Servo naturalBottomGripper, CRServo rotateServo, DigitalChannel ccwLimitSwitch, DigitalChannel cwLimitSwitch) {
        gripper1 = naturalTopGripper;
        gripper2 = naturalBottomGripper;
        upperGripper = gripper1;
        lowerGripper = gripper2;
        rotate = rotateServo;
        this.ccwLimitSwitch = ccwLimitSwitch;
        this.cwLimitSwitch = cwLimitSwitch;
    }

    /* @robot start: upperGripper is gripper1; lowerGripper is gripper2; */
    public boolean flip() {
        //todo don't allow flip unless move 3in up
        if (isFlipped) {
            if (rotate != null) {
                rotate.setDirection(DcMotor.Direction.REVERSE);
                rotate.setPower(1.0);
            }

            isFlipped = false;
            upperGripper = gripper1;
            lowerGripper = gripper2;
        } else {
            if (rotate != null) {
                rotate.setDirection(DcMotor.Direction.FORWARD);
                rotate.setPower(1.0);
            }

            isFlipped = true;

            upperGripper= gripper2;
            lowerGripper = gripper1;
        }

        return isFlipped;
    }

    public void lowerOpen() {
        lowerGripper.setPosition(GRIPPER_OPEN);
    }

    public void lowerClose() {
        lowerGripper.setPosition(GRIPPER_CLOSED);
    }

    public void upperOpen() {
        upperGripper.setPosition(GRIPPER_OPEN);
    }

    public void upperClose() {
        upperGripper.setPosition(GRIPPER_CLOSED);
    }

    public boolean isCWlimitReached() {
        if (cwLimitSwitch == null) {
            return false;
        }

        return !cwLimitSwitch.getState();
    }

    public boolean isCCWlimitReached() {
        if (ccwLimitSwitch == null) {
            return false;
        }

        return !ccwLimitSwitch.getState();
    }

    public void enforceLimits() {
        if (isCWlimitReached() || isCCWlimitReached()) {
            rotate.setPower(0D);
            Log.d(LOG_TAG, "limits reached, stopping servo");
        }
    }

    class Lift {
        private DcMotor lift;
        private DigitalChannel maxHeight;
        private DigitalChannel minHeight;

        private double upPower = 42;
        private double downPower = 11;
        public void stop(){
            lift.setPower(0);
        }

        public boolean moveUp(){
            lift.setPower(upPower);

            if (maxHeight.getState()) {
                lift.setPower(0);
                return false;
            }
            else{
                return true;
            }

        }

        public boolean moveDown() {
            lift.setPower(downPower);

            if (minHeight.getState()) {
                lift.setPower(0);
                return false;
            } else {
                return true;
            }
        }
    }
}

