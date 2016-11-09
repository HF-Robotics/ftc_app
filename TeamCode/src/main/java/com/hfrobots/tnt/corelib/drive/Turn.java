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

/**
 * A human-understandable description of a turn the robot must make during autonomous
 * with methods that are usable for steering with the gyro. Note that this class
 * follows the right-hand rule, so counter-clockwise turns are a positive relative
 * heading change
 */
public class Turn {
    private final RotationalDirection direction;
    private int degrees;
    private int asHeading;

    public Turn(RotationalDirection direction, int degrees) {
        this.direction = direction;
        this.degrees = degrees;

        final int headingSign;
        switch (this.direction) {
            case CLOCKWISE:
                headingSign = -1;
                break;
            case COUNTER_CLOCKWISE:
                headingSign = 1;
                break;
            default:
                throw new IllegalArgumentException("Illegal direction specified: " + direction);
        }

        asHeading = headingSign * degrees;
    }

    public int getHeading() {
        return asHeading;
    }

    /**
     * Returns an inverted representation of this Turn (i.e. for the opposite alliance)
     */
    public Turn invert() {
        switch (this.direction) {
            case CLOCKWISE:
                return new Turn(RotationalDirection.COUNTER_CLOCKWISE, degrees);
            case COUNTER_CLOCKWISE:
                return new Turn(RotationalDirection.CLOCKWISE, degrees);
            default:
                throw new IllegalArgumentException("Illegal direction specified: " + direction);
        }
    }

}
