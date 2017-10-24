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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

public class GlyphMechanism {
    private Servo upperGripper;
    private Servo lowerGripper;
    private Servo rotate;
    private Servo gripper1;
    private Servo gripper2;
    private boolean isFlipped;


    private static final double ROTATE_UNFLIPPED_POSITION= 42;
    private static final double ROTATE_FLIPPED_POSITION= 11;

    private static final double GRIPPER_OPEN = 0.5;
    private static final double GRIPPER_CLOSED = 0;

    public GlyphMechanism(Servo naturalTopGripper, Servo naturalBottomGripper, Servo rotateServo) {
        gripper1 = naturalTopGripper;
        gripper2 = naturalBottomGripper;
        upperGripper = gripper1;
        lowerGripper = gripper2;
        rotate = rotateServo;
    }

    /* @robot start: upperGripper is gripper1; lowerGripper is gripper2; */
    public boolean flip() {
        //todo don't allow flip unless move 3in up
        if (isFlipped) {
            if (rotate != null) {
               rotate.setPosition(ROTATE_UNFLIPPED_POSITION);
            }

            isFlipped = false;
            upperGripper = gripper1;
            lowerGripper = gripper2;
        }
        else {
            if (rotate != null) {
                rotate.setPosition(ROTATE_FLIPPED_POSITION);
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

