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

package com.hfrobots.tnt.corelib.control;

import java.util.concurrent.Callable;

public class DebouncedButton {
    final Callable<Boolean> getButtonState;
    final long waitBetweenPressesMillis;
    long buttonLastPressedMillis = 0;

    public DebouncedButton(Callable<Boolean> getButtonState, long waitBetweenPressesMillis) {
        this.getButtonState = getButtonState;
        this.waitBetweenPressesMillis = waitBetweenPressesMillis;
    }

    public boolean getDebouncedPressed() {
        long nowMillis = System.currentTimeMillis();

        if (buttonLastPressedMillis == 0 || nowMillis - buttonLastPressedMillis < waitBetweenPressesMillis) {
            try {
                boolean pressed = getButtonState.call().booleanValue();
                if (pressed) {
                    buttonLastPressedMillis = System.currentTimeMillis();
                }

                return pressed;
            } catch (Exception ex) {
                // this will never be thrown
                return false;
            }
        }

        return false; // we're waiting to debounce
    }
}
