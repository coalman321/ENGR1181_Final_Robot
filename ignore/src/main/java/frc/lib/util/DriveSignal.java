package frc.lib.util;

/**
 * A drivetrain command consisting of the linear and angular motor settings and whether the brake mode is enabled.
 */
public class DriveSignal {
    protected double mRightMotor;
    protected double mLeftMotor;
    protected boolean mBrakeMode;

    public static final DriveSignal NEUTRAL = new DriveSignal(0, 0, false);
    public static final DriveSignal BRAKE = new DriveSignal(0, 0, true);

    public DriveSignal(double left, double right, boolean brakeMode) {
        mLeftMotor = left;
        mRightMotor = right;
        mBrakeMode = brakeMode;
    }

    public double getLeft() {
        return mLeftMotor;
    }

    public double getRight() {
        return mRightMotor;
    }

    public boolean getBrakeMode() {
        return mBrakeMode;
    }

    @Override
    public String toString() {
        return "L: " + mLeftMotor + ", R: " + mRightMotor + (mBrakeMode ? ", BRAKE" : "");
    }
}