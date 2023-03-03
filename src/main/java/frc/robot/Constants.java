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
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 1; //  Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 12; //  Set back left steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(331.52109375000003); //  Measure and set back left steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 8; //  Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 7; //  Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 11; //  Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(86.042578125); //  Measure and set back right steer offset
    
    /* ELEVATOR CONSTANTS / PID VALUES */
    public static final int LEFT_ELEVATOR_MOTOR = 2; // Lead
    public static final int RIGHT_ELEVATOR_MOTOR = 5; // Follower

    public static final int ELEVATOR_LOW = 200;
    public static final int ELEVATOR_MID = 1200;
    public static final int ELEVATOR_HIGH = 1900;

    public static final double e_KP = 0.0008;
    public static final double e_KI = 4e-9;
    public static final double e_KD = 0;
    public static final double e_KF = 0.0001;
    public static final double e_OUTPUT_MIN = -2;
    public static final double e_OUTPUT_MAX = 2;

    /* ARM CONSTANTS */
    public static final int LEFT_ARM_MOTOR = 3; // Lead
    public static final int RIGHT_ARM_MOTOR = 4; // Follower

    public static final int ARM_STRAIGHT = 1300;
    public static final int ARM_RETRACT = 0;
    public static final int ARM_DOWN = 800;

    public static final double a_KP = 0.00054;
    public static final double a_KI = 3e-8;
    public static final double a_KD = 0.0;
    public static final double a_KF = 5e-5;
    public static final int a_OUTPUT_MAX = 2;
    public static final int a_OUTPUT_MIN = -2;
    public static final double a_MAX_VELOCITY = 0.0;
    public static final double a_MIN_VELOCITY = 0.0;
    public static final double a_MAX_ACCELERATION = 0.0;
    public static final double a_ALLOWED_ERROR = 0.0;

    /* INTAKE CONSTANTS */
    public static final int LEFT_INTAKE_MOTOR = 0;
    public static final int RIGHT_INTAKE_MOTOR = 0;

    public static final int LEFT_ACTUATOR = 0;
    public static final int RIGHT_ACTUATOR = 0;

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
