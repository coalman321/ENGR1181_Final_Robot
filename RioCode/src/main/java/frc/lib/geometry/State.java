package frc.lib.geometry;

import frc.lib.math.CSVWritable;
import frc.lib.math.Interpolable;

public interface State<S> extends Interpolable<S>, CSVWritable {
    double distance(final S other);

    boolean equals(final Object other);

    String toString();
}
