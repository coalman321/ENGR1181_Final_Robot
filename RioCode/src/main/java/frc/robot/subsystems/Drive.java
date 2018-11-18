package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.util.HIDHelper;
import frc.robot.Constants;

public class Drive extends Subsystem{

    private static final Drive M_DRIVE = new Drive();

    //system variables
    private DriveControlState mDriveControlState = DriveControlState.OPEN_LOOP;
    private double[] operatorInput = {0, 0, 0}; //last input set from joystick update

    //IO unit declarations

    private final Loop mLoop = new Loop(){

        @Override
        public void onStart(double timestamp) {
            synchronized (Drive.this){

            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Drive.this){
                if (Constants.ENABLE_MP_TEST_MODE && DriverStation.getInstance().isTest()) mDriveControlState = DriveControlState.PROFILING_TEST;
                switch(mDriveControlState){
                    case PATH_FOLLOWING:
                        if(Constants.RAMPUP){

                        }
                        else if(DriverStation.getInstance().isTest()){

                        }
                        break;
                    case PROFILING_TEST:

                        break;

                    case OPEN_LOOP:
                        if (DriverStation.getInstance().isOperatorControl()) operatorInput = HIDHelper.getAdjStick(Constants.MASTER_STICK);
                        else operatorInput = new double[]{0, 0, 0};
                        SmartDashboard.putNumberArray("stick", operatorInput);
                        //setOpenLoop(arcadeDrive(operatorInput[1], operatorInput[2]));
                        break;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            synchronized (Drive.this){

            }
        }
    };

    public static Drive getInstance(){
        return M_DRIVE;
    }

    @Override
    public void outputTelemetry() {

    }

    @Override
    public void stop() {

    }

    @Override
    public void reset() {

    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(mLoop);
    }

    enum DriveControlState {
        OPEN_LOOP,
        PATH_FOLLOWING,
        PROFILING_TEST;

        @Override
        public String toString() {
            return name().charAt(0) + name().substring(1).toLowerCase();
        }
    }
}