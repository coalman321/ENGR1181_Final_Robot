/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.lib.util.HIDHelper;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class Constants {
    // Drive CAN ID assignments
    public static final int DRIVE_FRONT_LEFT_ID = 0;
    public static final int DRIVE_FRONT_RIGHT_ID = 1;
    public static final int DRIVE_REAR_LEFT_ID = 2;
    public static final int DRIVE_REAR_RIGHT_ID = 3;

    //Sensor assignments
    public static final int PIGEON_ID = 0; //on canbus not via a talon

    //Robot Parameters


    //Right drive PID parameters
    public static final int DRIVE_RIGHT_PID_IDX = 0;
    public static final double DRIVE_RIGHT_KF = 0.0;
    public static final double DRIVE_RIGHT_KP = 0.0;
    public static final double DRIVE_RIGHT_KI = 0.0;
    public static final double DRIVE_RIGHT_KD = 0.0;

    //Left Drive PID parameters
    public static final int DRIVE_LEFT_PID_IDX = 0;
    public static final double DRIVE_LEFT_KF = 0.0;
    public static final double DRIVE_LEFT_KP = 0.0;
    public static final double DRIVE_LEFT_KI = 0.0;
    public static final double DRIVE_LEFT_KD = 0.0;

    //Logging Parameters
    public static final String DATA_SEPERATOR = "";
    public static final String[] NUMBER_KEYS = {};
    public static final String[] STRING_KEYS = {};


    public static final boolean ENABLE_MP_TEST_MODE = false;
    public static final boolean RAMPUP = false;

    public static final Joystick MASTER = new Joystick(0);
    public static final HIDHelper.HIDConstants MASTER_STICK = new HIDHelper.HIDConstants(MASTER, 0.15, 1.0, 1.0, 0.45, 2);
    public static final double LOOPER_DT = 0.010;
}
