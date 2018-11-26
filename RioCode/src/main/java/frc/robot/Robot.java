/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.loops.Looper;
import frc.lib.statemachine.StateMachine;
import frc.lib.util.Util;
import frc.robot.routines.DriveTest;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Logger;
import frc.robot.subsystems.PoseEstimator;

import java.util.Arrays;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    public static OI m_oi;
    public static SubsystemManager manager = new SubsystemManager(Arrays.asList(
            Drive.getInstance(),
            PoseEstimator.getInstance(),
            Logger.getInstance()
    ));
    private Looper disabledLooper = new Looper();
    private Looper enabledLooper = new Looper();


    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        Util.validateBuild();
        NetworkTableInstance.getDefault().setUpdateRate(Constants.LOOPER_DT);
        m_oi = new OI();
        Logger.getInstance().addNumberKeys(Constants.NUMBER_KEYS);
        Logger.getInstance().addStringKeys(Constants.STRING_KEYS);
        manager.registerEnabledLoops(enabledLooper);
        manager.registerDisabledLoops(disabledLooper);
        disabledLooper.start();

        SmartDashboard.putNumber("StateMachine/ state", -1);
        SmartDashboard.putString("drive/Markers passed", "");
    }

    /**
     * This function is called every robot packet, no matter the mode. Use
     * this for items like diagnostics that you want ran during disabled,
     * autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {

    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     * You can use it to reset any subsystem information you want to clear when
     * the robot is disabled.
     */
    @Override
    public void disabledInit() {
        enabledLooper.stop();
        disabledLooper.start();
    }

    @Override
    public void disabledPeriodic() {

    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * getString code to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional actions to the
     * chooser code above (like the commented example) or additional comparisons
     * to the switch structure below with additional strings & actions.
     */
    @Override
    public void autonomousInit() {
        manager.reset();
        enabledLooper.start();
        disabledLooper.stop();
        //TODO handle any communication between systems?
        //StateMachine.runMachine(new DriveTest());
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        manager.reset();
        enabledLooper.start();
        disabledLooper.stop();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void testInit() {
        manager.reset();
        enabledLooper.start();
        disabledLooper.stop();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {

    }
}
