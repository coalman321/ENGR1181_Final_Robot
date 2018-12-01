package frc.lib.trajectory;

import frc.lib.geometry.IPose2d;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Pose2dWithCurvature;
import frc.lib.spline.QuinticHermiteSpline;
import frc.lib.spline.Spline;
import frc.lib.spline.SplineGenerator;

import java.util.ArrayList;
import java.util.List;

public class TrajectoryUtil {

    public static <S extends IPose2d<S>> Trajectory<S> mirror(final Trajectory<S> trajectory) {
        List<S> waypoints = new ArrayList<>(trajectory.length());
        for (int i = 0; i < trajectory.length(); ++i) {
            waypoints.add(trajectory.getState(i).mirror());
        }
        return new Trajectory<>(waypoints);
    }

    public static <S extends IPose2d<S>> Trajectory<TimedState<S>> mirrorTimed(final Trajectory<TimedState<S>> trajectory) {
        List<TimedState<S>> waypoints = new ArrayList<>(trajectory.length());
        for (int i = 0; i < trajectory.length(); ++i) {
            TimedState<S> timed_state = trajectory.getState(i);
            waypoints.add(new TimedState<S>(timed_state.state().mirror(), timed_state.t(), timed_state.velocity(), timed_state.acceleration()));
        }
        return new Trajectory<>(waypoints);
    }

    public static <S extends IPose2d<S>> Trajectory<S> transform(final Trajectory<S> trajectory, Pose2d transform) {
        List<S> waypoints = new ArrayList<>(trajectory.length());
        for (int i = 0; i < trajectory.length(); ++i) {
            waypoints.add(trajectory.getState(i).transformBy(transform));
        }
        return new Trajectory<>(waypoints);
    }

    /**
     *
     * @param waypoints -- list of waypoints to generate from
     * @param maxDx -- maximum change in x between points
     * @param maxDy -- maximum change in y between points
     * @param maxDTheta -- maximum change in theta between points
     * @return
     */
    public static Trajectory<Pose2dWithCurvature> trajectoryFromSplineWaypoints(final List<Pose2d> waypoints, double
            maxDx, double maxDy, double maxDTheta) {
        List<QuinticHermiteSpline> splines = new ArrayList<>(waypoints.size() - 1);
        for (int i = 1; i < waypoints.size(); ++i) {
            splines.add(new QuinticHermiteSpline(waypoints.get(i - 1), waypoints.get(i)));
        }
        QuinticHermiteSpline.optimizeSpline(splines);
        return trajectoryFromSplines(splines, maxDx, maxDy, maxDTheta);
    }

    public static Trajectory<Pose2dWithCurvature> trajectoryFromSplines(final List<? extends Spline> splines, double
            maxDx, double maxDy, double maxDTheta) {
        return new Trajectory<>(SplineGenerator.parameterizeSplines(splines, maxDx, maxDy, maxDTheta));
    }

}
