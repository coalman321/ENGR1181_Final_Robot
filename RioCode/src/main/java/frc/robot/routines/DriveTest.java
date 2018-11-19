package frc.robot.routines;

import frc.lib.AutoTrajectory.Path;
import frc.lib.AutoTrajectory.Translation2d;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.FollowPath;

import java.util.ArrayList;
import java.util.List;

public class DriveTest extends StateMachineDescriptor {

    public DriveTest() {
        //create trajectory
        List<Path.Waypoint> traj = new ArrayList<>();
        traj.add(new Path.Waypoint(new Translation2d(0, 0), 40));
        traj.add(new Path.Waypoint(new Translation2d(120, 0), 40)); // +x is forward and +y is right

        //create state descriptor
        addSequential(new FollowPath(new Path(traj), false), 10000); // 10 seconds
    }

}
