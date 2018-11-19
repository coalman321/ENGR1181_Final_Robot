package frc.lib.statemachine;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.concurrent.ConcurrentLinkedQueue;

public class StateMachine {

    private volatile static int state = -1;
    private volatile static ConcurrentLinkedQueue<ActionGroup> queuedStates;
    private volatile static ActionGroup currentState;
    private volatile static double t_start;
    private static Runnable Man = () -> {
        state = 0;
        SmartDashboard.putNumber("StateMachine/ state", state);
        if (queuedStates == null) {
            state = -2;
        } else {
            while (!queuedStates.isEmpty()) {
                currentState = queuedStates.poll();
                currentState.onStart();
                while (!currentState.isFinnished()) {
                    t_start = Timer.getFPGATimestamp();
                    currentState.onLoop();
                    Timer.delay(0.02 - (Timer.getFPGATimestamp() - t_start));
                }
                currentState.onStop();
                state++;

            }
        }
    };

    public static void runMachine(StateMachineDescriptor descriptor) {
        queuedStates = descriptor.getStates();
        Thread thread = new Thread(Man);
        thread.start();

    }


}
