package frc.lib.statemachine;

import java.util.concurrent.ConcurrentLinkedQueue;

public class StateMachineDescriptor {
    private ConcurrentLinkedQueue<ActionGroup> queuedStates;

    public StateMachineDescriptor() {
        queuedStates = new ConcurrentLinkedQueue<>();
    }

    public void addSequential(Action action, long timeout_ms) {
        queuedStates.add(new ActionGroup(action, timeout_ms));
    }

    public void addParallel(Action[] action, long timeout_ms) {
        queuedStates.add(new ActionGroup(action, timeout_ms));
    }

    public ConcurrentLinkedQueue getStates() {
        return queuedStates;
    }
}
