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
package com.hfrobots.tnt.corelib.state;

import com.hfrobots.tnt.corelib.control.DebouncedButton;
import com.hfrobots.tnt.corelib.control.DebouncedGamepadButtons;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Set;
import java.util.Stack;

public class StateMachine {

    private Stack<State> executedStates;

    private Set<State> allStates;

    private State currentState = null;

    private State firstState;

    private boolean areWeDebugging = false;

    private boolean isStateMachinePaused = false;

    private DebouncedButton goButton;

    private DebouncedButton goBackButton;

    private DebouncedButton doOverButton;

    private DebouncedGamepadButtons allGamePadButtons;

    private Telemetry telemetry;

    public void addNewState(State newState) {
        allStates.add(newState);
    }

    public void startDebugging() {
        areWeDebugging = true;
    }

    public void stopDebugging() {
        areWeDebugging = false;
    }

    /**
     * Sets the first State the state machine will use.
     *
     * If the state has not already been added w/ addNewState then it will added for you
     *
     * @param state The first state the state machine will execute
     * @throws  IllegalStateException if this method has already been called
     */
    public void setFirstState(State state) {
        if (currentState != null) {
            throw new IllegalArgumentException("State machine already has the first state set");
        }

        executedStates.push(state);
        addNewState(state);
    }

    public void doOneStateLoop() {
        if (!isStateMachinePaused) {
            State possibleNextState = currentState.doStuffAndGetNextState();

            if (!possibleNextState.equals(currentState)) {
                // We've changed states, Yay time to party
                executedStates.push(possibleNextState);
                currentState = possibleNextState;

                if (areWeDebugging) {
                    isStateMachinePaused = true;
                }
            }
        } else {
            // we're paused, allowing live configuring and waiting for go or go back signals
            currentState.liveConfigure(allGamePadButtons);

            // check for un-pausing
            if (goButton.getRise()) {
                isStateMachinePaused = false;
            } else if (goBackButton.getRise()) {
                currentState = executedStates.pop();
                currentState.resetToStart();
                isStateMachinePaused = true;
            } else if (doOverButton.getRise()) {
                // reset all the states, set current to ??? and pause the state machine
                for (State resetThisState : allStates) {
                    resetThisState.resetToStart();
                }

                executedStates.clear();

                currentState = firstState;
                isStateMachinePaused = true;
            }
        }

        telemetry.addData("00", String.format("%s%s state %s", areWeDebugging ? "[DEBUG]" : "",
                isStateMachinePaused ? "||" : ">", currentState.getName()));
    }

}
