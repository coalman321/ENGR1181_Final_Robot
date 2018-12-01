package frc.lib.trajectory;

import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Pose2dWithCurvature;
import frc.lib.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class TrajectoryCreationTest {

    @Test
    public void creationTest() {
        System.out.println("Running trajectory from splines waypoints test:");
        List<Pose2d> waypts = Arrays.asList(Pose2d.identity(), new Pose2d(20, 20, Rotation2d.fromDegrees(0)), new Pose2d(100, 20, Rotation2d.fromDegrees(90)));
        Trajectory<Pose2dWithCurvature> trajectory = TrajectoryUtil.trajectoryFromSplineWaypoints(waypts, 10, 10, 3.14);
        for(int i = 0; i < trajectory.length(); i++){
            System.out.println(trajectory.getState(i).toString());
        }
        assertEquals(trajectory.getLastState().getPose(), new Pose2d(100, 20, Rotation2d.fromDegrees(90)));
    }


}
