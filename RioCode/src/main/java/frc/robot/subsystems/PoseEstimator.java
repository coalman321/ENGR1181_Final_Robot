package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.lib.geometry.Twist2d;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.math.InterpolatingDouble;
import frc.lib.math.InterpolatingTreeMap;
import frc.robot.Constants;
import frc.robot.Kinematics;

import java.util.Map;


public class PoseEstimator extends Subsystem {

    private static PoseEstimator m_instance = new PoseEstimator();

    private static final int observation_buffer_size = 100;

    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> fieldToVehicle;
    private double linear_prev_dist = 0;
    private double distance_driven = 0;

    private Loop mLoop = new Loop(){

        @Override
        public void onStart(double timestamp) {
            linear_prev_dist = 0;
            distance_driven = 0.0;
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (this){
                final Rotation2d gyro = Drive.getInstance().getHeading();
                final double linear_distance = Drive.getInstance(). //TODO implement
                final double linear_delta = linear_distance - linear_prev_dist;
                Twist2d odometry_velocity = generateOdometryFromSensors(linear_delta, gyro);
                addObservations(timestamp, Kinematics.integrateForwardKinematics(getLatestFieldToVehicle().getValue(), odometry_velocity));
                linear_prev_dist = linear_distance;
                outputTelemetry();
            }
        }

        @Override
        public void onStop(double timestamp) {

        }
    };

    public static PoseEstimator getInstance(){
        return m_instance;
    }

    private PoseEstimator(){
        reset(0, Pose2d.identity());
    }

    public void reset(double start_time, Pose2d initial_field_to_vehicle){
        fieldToVehicle = new InterpolatingTreeMap<>(observation_buffer_size);
        fieldToVehicle.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
        return fieldToVehicle.lastEntry();
    }

    public synchronized Twist2d generateOdometryFromSensors(double x_delta_distance, Rotation2d current_gyro_angle) {
        final Pose2d last_measurement = getLatestFieldToVehicle().getValue();
        final Twist2d delta = Kinematics.forwardKinematics2(x_delta_distance, last_measurement.getRotation(), current_gyro_angle);
        return delta;
    }

    public synchronized void addObservations(double timestamp, Pose2d observation) {
        fieldToVehicle.put(new InterpolatingDouble(timestamp), observation);
    }


    @Override
    public void outputTelemetry() {
        Pose2d odometry = getLatestFieldToVehicle().getValue();
        SmartDashboard.putNumber("Robot Pose/X", odometry.getTranslation().x());
        SmartDashboard.putNumber("Robot Pose/Y", odometry.getTranslation().y());
        SmartDashboard.putNumber("Robot Pose/Theta", odometry.getRotation().getDegrees());
    }

    @Override
    public void stop() {

    }

    @Override
    public void reset() {
        reset(Timer.getFPGATimestamp(), Pose2d.identity());
    }

    @Override
    public void registerEnabledLoops(ILooper looper){
        looper.register(mLoop);
    }
}
