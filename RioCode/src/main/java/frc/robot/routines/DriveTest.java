package frc.robot.routines;

import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.lib.trajectory.Trajectory;

import java.util.Arrays;
import java.util.List;


public class DriveTest extends StateMachineDescriptor {

    public static final List<Pose2d> waypoints = Arrays.asList(new Pose2d(0,0, Rotation2d.fromDegrees(0)),
                                                                new Pose2d(10, 0, Rotation2d.fromDegrees(0)),
                                                                new Pose2d(100, 0, Rotation2d.fromDegrees(90)));

    public DriveTest() {
        Trajectory<Pose2d> tra = new Trajectory<>(waypoints);
    }

}
