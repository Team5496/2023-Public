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
    // DRIVETRAIN CONSTANTS
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


    // ELEVATOR CONSTANTS
    public static final int LEAD_ELEVATOR_ID = 23; // Left
    public static final int FOLLOW_ELEVATOR_ID = 25; // Right

    public static final double e_KP = 0.0005;
    public static final double e_KI = 0.0;
    public static final double e_KD = 0.010;
    public static final double e_KF = 0.00026;
    public static final double e_OUTPUT_MIN = -2;
    public static final double e_OUTPUT_MAX = 2;

    public static final int ELEVATOR_CARRY = 200;
    public static final int ELEVATOR_SHELF = 700;

    public static final int ELEVATOR_HIGH_CONE = 1700;
    public static final int ELEVATOR_MID_CONE = 1500;
    
    public static final int ELEVATOR_HIGH_CUBE = 1600;
    public static final int ELEVATOR_MID_CUBE = 760;


    // ARM CONSTANTS
    public static final int ARM_ID = 21; // Lead
    
    public static final double a_KP = 0.00032; 
    public static final double a_KI = 0.0;
    public static final double a_KD = 0.0; 
    public static final double a_KF = 0.0;

    public static final int a_OUTPUT_MAX = 2;
    public static final int a_OUTPUT_MIN = -2;
    public static final double a_MAX_VELOCITY = 0.0;
    public static final double a_MIN_VELOCITY = 0.0;
    public static final double a_MAX_ACCELERATION = 0.0;
    public static final double a_ALLOWED_ERROR = 0.0;
    
    public static final int ARM_CARRY = 0;

    public static final int ARM_HIGH_CONE = -6360;
    public static final int ARM_MID_CONE = -2920;
    public static final int ARM_LOW_CONE = 0;

    public static final int ARM_HIGH_CUBE = -5150;
    public static final int ARM_MID_CUBE = -1720;
    public static final int ARM_LOW_CUBE = 0;
   

    // INTAKE ARM CONSTANTS
    public static final int INTAKE_ARM_ID = 14; // Falcon
    public static final double ia_KP = 0.03;
    public static final double ia_KI = 0.0;
    public static final double ia_KD = 0.05;
    public static final double ia_KF = 0.01;

    public static final int VTICKS = -24000;
    public static final int HTICKS = -97800;

    public static final int INARM_CARRY = 0;

    public static final int INARM_HIGH_CONE = -38290;
    public static final int INARM_MID_CONE = -45760;
    public static final int INARM_LOW_CONE = 345; // Need to Check
    public static final int INARM_FLOOR_CONE = -61180;
    public static final int INARM_RAMP_CONE = -21465;    

    public static final int INARM_HIGH_CUBE = -52700;
    public static final int INARM_MID_CUBE = -44990;
    public static final int INARM_LOW_CUBE = 0; // Need to Check
    public static final int INARM_FLOOR_CUBE = 0; // Need to Check
    public static final int INARM_RAMP_CUBE = -18385;


    // INTAKE SWIVEL CONSTANTS
    public static final int SWIVEL_ID = 26; // 775

    public static final double s_P = 0.5; // TODO: Test values
    public static final double s_I = 0.00;
    public static final double s_D = 0.0;
    public static final double s_F = 0.001;

    // FULL ROTATION = 6500 TICKS
    public static final int SWIVEL_CARRY = 2700;

    public static final int SWIVEL_HIGH_CONE = 5385;
    public static final int SWIVEL_MID_CONE = -810;
    public static final int SWIVEL_LOW_CONE = -1070; // Need to Check
    public static final int SWIVEL_FLOOR_CONE = 4245; // Need to Check
    public static final int SWIVEL_RAMP_CONE = 4245;

    public static final int SWIVEL_HIGH_CUBE = -3955;
    public static final int SWIVEL_MID_CUBE = 2790;
    public static final int SWIVEL_LOW_CUBE = 2645; // Need to Check
    public static final int SWIVEL_FLOOR_CUBE = 1900; // Need to Check
    public static final int SWIVEL_RAMP_CUBE = 1900;


    // INTAKE CONSTANTS
    public static final int INTAKE_ID = 28; // Neo

    public static final double INTAKE_FORWARD = 0.8;
    public static final double INTAKE_BACKWARD = -0.8;

    public static final double INTAKE_FORWARD_HOLD = 0.08;
    public static final double INTAKE_BACKWARD_HOLD = -0.08;

    
    // VISION CONSTANTS
    public static final double CAMERA_HEIGHT = 0.0;
    public static final double CAMERA_PITCH = 0.0;

    public static final double GRID_TARGET_HEIGHT = 0.0;
    public static final double LOADING_TARGET_HEIGHT = 0.0;


    // LIGHT CONTANTS
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
