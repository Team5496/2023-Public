// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import frc.robot.subsystems.LightStates;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.6223; //  Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.6223; //  Measure and set wheelbase

    public static final int DRIVETRAIN_PIGEON_ID = 13; //  Set Pigeon ID

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 2; //  Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 4; //  Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 10; //  Set front left steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(342.06210937500003); //  Measure and set front left steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 5; //  Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 3; //  Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 9; //  Set front right steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(204.52031250000002); //  Measure and set front right steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 6; //  Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 22; //  Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 12; //  Set back left steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(331.52109375000003); //  Measure and set back left steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 8; //  Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 7; //  Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 11; //  Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(86.042578125); //  Measure and set back right steer offset
    
    public static final double WHEEL_CIRCUMFERENCE = 12.57;

    /* ELEVATOR CONSTANTS / PID VALUES */
    public static final int LEFT_ELEVATOR_MOTOR = 23; // Lead
    public static final int RIGHT_ELEVATOR_MOTOR = 25; // Follower

    public static final int ELEVATOR_LOW = 400;
    public static final int ELEVATOR_MID = 850;
    public static final int ELEVATOR_SHELF = 700;
    public static final int ELEVATOR_PICK_UP = 950;
    public static final int ELEVATOR_HIGH = 2000;

    public static final double e_KP = 0.00072;
    public static final double e_KI = 0.0;
    public static final double e_KD = 0.00135;
    public static final double e_KF = 0.000095;
    public static final double e_OUTPUT_MIN = -2;
    public static final double e_OUTPUT_MAX = 2;

    /* ARM CONSTANTS */
    public static final int ARM_MOTOR = 21; // Lead
    public static final int ARM_STRAIGHT = -3400; //1400
    public static final int ARM_RETRACT = -100; //-300

    public static final double a_KP = 0.00032; 
    public static final double a_KI = 0.0; //3e-10
    public static final double a_KD = 0.0; 
    public static final double a_KF = 0.0; //2e-6

    public static final double a_KPR = 0.00030; //0.001
    public static final double a_KIR = 8.75e-11; //3.5e-10
    public static final double a_KDR = 0.0;
    public static final double a_KFR = 5e-7; //2e-6

    public static final int a_OUTPUT_MAX = 2;
    public static final int a_OUTPUT_MIN = -2;
    public static final double a_MAX_VELOCITY = 0.0;
    public static final double a_MIN_VELOCITY = 0.0;
    public static final double a_MAX_ACCELERATION = 0.0;
    public static final double a_ALLOWED_ERROR = 0.0;

    /* INTAKE ARM CONSTANTS */
    public static final int INTAKE_ARM_ID = 14;  //falcon
    public static final double IA_KP = 0.0111; // 0.09
    public static final double IA_KI = 0.0; // 0
    public static final double IA_KD = 0.111; // 6.78
    public static final double IA_KF = 0.0495; // 0.03


    public static final int IA_IN = -100;
    public static final int IA_OUT = -20000;

    public static final int VTICKS = -24000;
    public static final int HTICKS = -97800;

    /*
    public static final double IA_KP_BACKUP = 0.021408;
    public static final double IA_KI_BACKUP = 0;
    public static final double IA_KD_BACKUP = 0;
    public static final double IA_KF_BACKUP = 0.031972;
    */

    /* INTAKE CONSTANTS */
    public static final int INTAKE_MOTOR = 0; //775
    public static final double INTAKE_IN = 0.45;
    public static final double INTAKE_OUT = 0.45;

    /* INTAKE SWIVEL CONSTANTS */
    public static final double s_P = 0.1; // TODO: Test values
    public static final double s_I = 0.001;
    public static final double s_D = 1;
    public static final double s_F = 0.1;
    public static final int SwivelCANID = 2; //neo
    
    /* VISION CONSTANTS */
    public static final double CAMERA_HEIGHT = 0.0;
    public static final double CAMERA_PITCH = 0.0;

    public static final double GRID_TARGET_HEIGHT = 0.0;
    public static final double LOADING_TARGET_HEIGHT = 0.0;

    public static final Map<LightStates,String> LIGHT_STATES; 
    public static final String LIGHT_STATE_OFF =  "{'on':false}";
    public static final String LIGHT_STATE_ON =  "{'on':true}";
    public static final String DEFAULT_LIGHT_STATE = LIGHT_STATE_OFF;
    static {
        LIGHT_STATES = new HashMap<>();
        LIGHT_STATES.put(LightStates.OFF,LIGHT_STATE_OFF);
        LIGHT_STATES.put(LightStates.ON, LIGHT_STATE_ON);   
    }
} 
