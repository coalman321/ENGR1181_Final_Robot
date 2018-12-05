package frc.robot.actions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Drive;

public class PathCompletionWait extends Action {
    @Override
    public void onStart() {

    }

    @Override
    public void onLoop() {

    }

    @Override
    public boolean isFinished() {
        return Drive.getInstance().isFinishedPath();
    }

    @Override
    public void onStop() {

    }
}
