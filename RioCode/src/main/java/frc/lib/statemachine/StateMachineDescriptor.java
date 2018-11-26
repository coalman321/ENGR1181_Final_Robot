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

    public void addParallel(Action[] actions, long timeout_ms) { queuedStates.add(new ActionGroup(actions, timeout_ms));
    }

    public ConcurrentLinkedQueue<ActionGroup> getStates() {
        return queuedStates;
    }
}
