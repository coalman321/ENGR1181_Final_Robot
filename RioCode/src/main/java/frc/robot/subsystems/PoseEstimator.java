package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.AutoTrajectory.*;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.robot.Constants;

import java.util.Map;


public class PoseEstimator extends Subsystem {

    private static PoseEstimator m_instance = new PoseEstimator();

    private InterpolatingTreeMap<InterpolatingDouble, RigidTransform2d> fieldToVehicle;
    private double leftPrevEncCount = 0;
    private double rightPrevEncCount = 0;

    private Loop mLoop = new Loop(){

        @Override
        public void onStart(double timestamp) {

        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (this){
                double currentTime = Timer.getFPGATimestamp();
                double currentLeftEncoder = Drive.getInstance().getLeftEncoderRotations() * Constants.WHEEL_DIAMETER * 3.14159;
                double currentRightEncoder = Drive.getInstance().getRightEncoderRotations() * Constants.WHEEL_DIAMETER * 3.14159;
                Rotation2d gyro = Drive.getInstance().getHeading();
                RigidTransform2d odometry = generateOdometryFromSensors((currentLeftEncoder - leftPrevEncCount), (currentRightEncoder - rightPrevEncCount), gyro);
                addObservations(currentTime, odometry);
                leftPrevEncCount = currentLeftEncoder;
                rightPrevEncCount = currentRightEncoder;
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
        reset(0, new RigidTransform2d());
    }

    public void reset(double start_time, RigidTransform2d initial_field_to_vehicle){
        fieldToVehicle = new InterpolatingTreeMap<>(Constants.OBSERVATION_BUFFER_SIZE);
        fieldToVehicle.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
        leftPrevEncCount = 0;
        rightPrevEncCount = 0;
    }

    public synchronized Map.Entry<InterpolatingDouble, RigidTransform2d> getLatestFieldToVehicle() {
        return fieldToVehicle.lastEntry();
    }

    public RigidTransform2d generateOdometryFromSensors(double left_encoder_delta_distance, double right_encoder_delta_distance, Rotation2d current_gyro_angle) {
        RigidTransform2d last_measurement = getLatestFieldToVehicle().getValue();
        return Kinematics.integrateForwardKinematics(last_measurement, left_encoder_delta_distance, right_encoder_delta_distance, current_gyro_angle);
    }

    public synchronized void addObservations(double timestamp, RigidTransform2d observation) {
        fieldToVehicle.put(new InterpolatingDouble(timestamp), observation);
    }


    @Override
    public void outputTelemetry() {
        RigidTransform2d odometry = getLatestFieldToVehicle().getValue();
        SmartDashboard.putNumber("Robot Pose/ X", odometry.getTranslation().getX());
        SmartDashboard.putNumber("Robot Pose/ Y", odometry.getTranslation().getY());
        SmartDashboard.putNumber("Robot Pose/ Theta", odometry.getRotation().getDegrees());
    }

    @Override
    public void stop() {

    }

    @Override
    public void reset() {
        reset(Timer.getFPGATimestamp(), new RigidTransform2d());
    }

    @Override
    public void registerEnabledLoops(ILooper looper){
        looper.register(mLoop);
    }
}
