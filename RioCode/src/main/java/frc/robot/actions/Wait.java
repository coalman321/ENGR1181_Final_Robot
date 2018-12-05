package frc.robot.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.statemachine.Action;

public class Wait extends Action {

    public Wait(double delay)

    double t_start = 0, t_delay = 0;

    @Override
    public void onStart() {
        t_start = Timer.getFPGATimestamp();
    }

    @Override
    public void onLoop() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void onStop() {

    }
}
