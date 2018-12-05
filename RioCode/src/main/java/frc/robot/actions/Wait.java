package frc.robot.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.statemachine.Action;

public class Wait extends Action {

    double t_start = 0, t_delay, t_end = 0;

    public Wait(double delay){
        t_delay = delay;
    }

    @Override
    public void onStart() {
        t_start = Timer.getFPGATimestamp();
        t_end = t_start + t_delay;
    }

    @Override
    public void onLoop() {

    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() > t_end;
    }

    @Override
    public void onStop() {

    }
}
