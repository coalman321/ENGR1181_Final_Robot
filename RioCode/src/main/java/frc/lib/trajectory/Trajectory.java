package frc.lib.trajectory;

import frc.lib.geometry.State;

import java.util.ArrayList;
import java.util.List;

public class Trajectory<S extends State<S>> {
    protected final List<TrajectoryPoint<S>> points_;

    /**
     * Create an empty trajectory.
     */
    public Trajectory() {
        points_ = new ArrayList<TrajectoryPoint<S>>();
    }

    /**
     * Create a trajectory from the given states and transforms.
     *
     * @param states The states of the trajectory.
     */
    public Trajectory(final List<S> states) {
        points_ = new ArrayList<>(states.size());
        for (int i = 0; i < states.size(); ++i) {
            points_.add(new TrajectoryPoint<>(states.get(i), i));
        }
    }

    public boolean isEmpty() {
        return points_.isEmpty();
    }

    public int length() {
        return points_.size();
    }

    public TrajectoryPoint<S> getPoint(final int index) {
        return points_.get(index);
    }

    public S getState(final int index) {
        return getPoint(index).state();
    }

    public S getFirstState() {
        return getState(0);
    }

    public S getLastState() {
        return getState(length() - 1);
    }

    public TrajectorySamplePoint<S> getInterpolated(final double index) {
        if (isEmpty()) {
            return null;
        } else if (index <= 0.0) { //if under index goto first
            return new TrajectorySamplePoint<>(getPoint(0));
        } else if (index >= length() - 1) { // if above length goto last
            return new TrajectorySamplePoint<>(getPoint(length() - 1));
        }
        final int i = (int) Math.floor(index);
        final double frac = index - i; // get just decimals
        if (frac <= Double.MIN_VALUE) { //if less than min value of double return the trajectorypoint of index
            return new TrajectorySamplePoint<>(getPoint(i));
        } else if (frac >= 1.0 - Double.MIN_VALUE) {
            return new TrajectorySamplePoint<>(getPoint(i + 1));
        } else {
            return new TrajectorySamplePoint<>(getState(i).interpolate(getState(i + 1), frac), i, i + 1);
        }
    }


    @Override
    public String toString() {
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < length(); ++i) {
            builder.append(i);
            builder.append(": ");
            builder.append(getState(i));
            builder.append(System.lineSeparator());
        }
        return builder.toString();
    }

}
