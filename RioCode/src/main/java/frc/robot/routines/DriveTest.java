package frc.robot.routines;

import frc.lib.AutoTrajectory.Path;
import frc.lib.AutoTrajectory.Translation2d;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.FollowPath;
import frc.robot.actions.PathCompletionWait;
import frc.robot.actions.Wait;

import java.util.ArrayList;
import java.util.List;

public class DriveTest extends StateMachineDescriptor {

    public DriveTest() {

        double speedLimit = 10;
        //create trajectory
        List<Path.Waypoint> traj = new ArrayList<>();
        traj.add(new Path.Waypoint(new Translation2d(0, 0), speedLimit));
        traj.add(new Path.Waypoint(new Translation2d(30, 0), speedLimit));
        traj.add(new Path.Waypoint(new Translation2d(60,-20), speedLimit));
        traj.add(new Path.Waypoint(new Translation2d(90, 0), speedLimit));
        traj.add(new Path.Waypoint(new Translation2d(120, 0), speedLimit)); // +x is forward and +y is left

        List<Path.Waypoint> rev = new ArrayList<>();
        rev.add(new Path.Waypoint(new Translation2d(120, 0), speedLimit));
        rev.add(new Path.Waypoint(new Translation2d(90, 0), speedLimit));
        rev.add(new Path.Waypoint(new Translation2d(60,-20), speedLimit));
        rev.add(new Path.Waypoint(new Translation2d(30, 0), speedLimit));
        rev.add(new Path.Waypoint(new Translation2d(0, 0), speedLimit)); // +x is forward and +y is left

        //create state descriptor
        addSequential(new FollowPath(new Path(traj), false), 15000); // 10 seconds
        addSequential(new PathCompletionWait(), 10000);
        addSequential(new Wait(1.0), 1000);
        addSequential(new FollowPath(new Path(rev), true), 15000); // 10 seconds
    }

}
