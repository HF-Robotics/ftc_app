package com.hfrobots.tnt.corelib.drive;

import com.hfrobots.tnt.corelib.state.State;
import com.hfrobots.tnt.corelib.state.TimeoutSafetyState;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class TurnState extends TimeoutSafetyState {
    private static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    private static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable

    private final TankDrive drive;
    private final ModernRoboticsI2cGyro gyro;
    private final Turn turn;
    private final double initialPower;

    protected TurnState(TankDrive drive,
                        ModernRoboticsI2cGyro gyro,
                        Turn turn,
                        Telemetry telemetry,
                        double initialPower,
                        long safetyTimeoutMillis) {
        super(telemetry, safetyTimeoutMillis);
        this.drive = drive;
        this.gyro = gyro;
        this.turn = turn;
        this.initialPower = initialPower;
    }

    @Override
    public State doStuffAndGetNextState() {
        // our implementation is based on relative angles, so we need to compute that here, how?

        // this is a linear op mode example, how do we change this to fit into our state machine?
        // keep looping while we are still active, and not on heading.
        while (/* opModeIsActive() */ !onHeading(initialPower, 0 /* angle */)) {
            // Update telemetry & Allow time for other processes to run.
            //telemetry.update();
        }

        return null;
    }

    // re-use of Pushbot gyro steer

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @return
     */
    boolean onHeading(double speed, double angle) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            // Note - this is true tank steering, one side moves directly opposite the other!
            steer = getSteer(error);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        drive.drivePower(leftSpeed, rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @return
     */
    public double getSteer(double error) {
        return Range.clip(error * P_TURN_COEFF, -1, 1);
    }
}
