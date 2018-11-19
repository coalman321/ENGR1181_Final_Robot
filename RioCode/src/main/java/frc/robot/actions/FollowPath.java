package frc.robot.actions;

import frc.lib.AutoTrajectory.Path;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Drive;

public class FollowPath extends Action {

    private Path mPath;
    private boolean mReversed, mHasStarted;

    public FollowPath(Path path, boolean reversed){
        mPath = path;
        mReversed = reversed;
        mHasStarted = false;
    }


    @Override
    public void onStart() {
        Drive.getInstance().followPath(mPath, mReversed);
    }

    @Override
    public void onLoop() {
        mHasStarted = true;
    }

    @Override
    public boolean isFinished() {
        boolean done = Drive.getInstance().isFinishedPath() && mHasStarted;
        if (done) {
            System.out.println("Finished path");
        }
        return done;
    }

    @Override
    public void onStop() {

    }
}
