package com.hfrobots.tnt.season1819.support;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.trajectory.DashboardUtil;
import com.acmerobotics.roadrunner.followers.MecanumPIDVAFollower;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.hfrobots.tnt.season1819.RoadrunnerMecanumDriveAdapter;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Spline follow", group="Utilities")
public class SplineFollowOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //FtcDashboard dashboard = FtcDashboard.getInstance();
        RoadrunnerMecanumDriveAdapter drive = new RoadrunnerMecanumDriveAdapter(hardwareMap);
        // change these constraints to something reasonable for your drive
        DriveConstraints baseConstraints = new DriveConstraints(40.0, 60.0, Math.PI / 2, Math.PI / 2);
        MecanumConstraints constraints = new MecanumConstraints(baseConstraints, drive.getTrackWidth(), drive.getWheelBase());
        //Trajectory trajectory = new TrajectoryBuilder(new Pose2d(0, 0, 0), constraints)
        //        .splineTo(new Pose2d(12, 12, 0)).build();

        Trajectory trajectory = new TrajectoryBuilder(new Pose2d(0, 0, 0), constraints)
                .lineTo(new Vector2d(12, 0))
                .build();

        Log.d("TNT", "Trajectory duration: " + trajectory.duration());

        // TODO: tune kV, kA, and kStatic in the following follower
        // then tune the PID coefficients after you verify the open loop response is roughly correct
        MecanumPIDVAFollower follower = new MecanumPIDVAFollower(
                drive,
                new PIDCoefficients(0, 0, 0),
                new PIDCoefficients(0, 0, 0),
                .01283,
                0,
                0);

        waitForStart();

        follower.followTrajectory(trajectory);
        while (opModeIsActive() && follower.isFollowing()) {
            Pose2d currentPose = drive.getPoseEstimate();

            Log.d("TNT", "Pose: " + currentPose.getX() + ", " + currentPose.getY() + ", " + currentPose.getHeading());
            Log.d("TNT", "Remaining duration: " + trajectory.duration());

            //TelemetryPacket packet = new TelemetryPacket();
            //Canvas fieldOverlay = packet.fieldOverlay();
            //fieldOverlay.setStroke("green");
            //DashboardUtil.drawSampledTrajectory(fieldOverlay, trajectory);
            //fieldOverlay.setFill("blue");
            //fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);
            //dashboard.sendTelemetryPacket(packet);

            follower.update(currentPose);
            drive.updatePoseEstimate();
        }
    }
}
