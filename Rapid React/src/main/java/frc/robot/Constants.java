// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final class DriveConstants{
        /**
         * The DriveConstants class is used to define all of the constants we use for driving. 
         * These include the IDs of the motors, conversion rates from tics to feet, etc.
         * To call this, call from the constants file as you normally would, but instead of doing
         * Constants.whateveryouwanttocall say DriveConstants.whateveryouwanttocall
         * IDS are the ports the motors are connected to
         */
        public static final int FALCON_FR_ID = 47;
        public static final int FALCON_FL_ID = 48;
        public static final int FALCON_BR_ID = 46;
        public static final int FALCON_BL_ID = 49;
        

        public static final double DRIVE_P = 4; /**not sure what these do TODO ask mikey but im pretty sure theyre the offset
         for the drive forward but yeah so like do we need to change this Mikey? If you see this remind me to ask them, it was 
         not documented in Annie's code*/
        public static final double DRIVE_I = 1;

        //public static final double TIC_FT = ((Math.PI)/2014)/10.75; (Is this accurate still?)


    }

    public final class ShooterConstants
    {
        public static final int NEO_TURRET_ID = 52;
        public static final float Turret_Kp = -0.005f; //speed modifier of le turret
        public static final float min_command = 0.00001f; //deadzone of le turret
        public static final double hub_height = 104 ; //inches 
        public static final double limelight_height = 9.125; //inches
        public static final double limelight_angle = 33; //degrees
        public static final int FALCON_shooter_ID = 0; //placeholder
        public static final int kP = 0; //placeholder position constant
        public static final int kA = 0; //placeholder acceleration constant of shooter
        public static final int kV = 0; //placeholder describes how much voltage is needed to hold (or “cruise”) at a given constant velocity while overcoming the electromagnetic resistance in the motor and any additional friction that increases with speed (known as viscous drag). The relationship between speed and voltage (at constant acceleration) is almost entirely linear
        public static final int kS = 0; //placeholder  voltage needed to overcome the motor’s static friction
        public static final int hood_encoder_ratio = 0; //ratio of encoder count to degree angle of turret, placeholder
        public static final int hood_motor_ID = 0; //placeholder
        public static final double hood_min_command = .05;
    }

    public final class OIConstants {
        public static final int XBOX_ID = 0;
        public static final int HELMS_ID = 1;
    
    }

    public final class IntakeConstants
    {
        public static final int INTAKE_ID = 8; //placeholder
        public static final double INTAKE_SPEED = 0.5; //placeholder
        public static final int INTAKE_DROPDOWN_ID = 0; //placeholder
        public static final double DROPDOWN_SPEED = 0.5; //placeholder
    }

    public final class ElevatorConstants
    {
        public static final int FRONT_ELEVATOR_ID = 8; //placeholder
        public static final int BACK_ELEVATOR_ID = 7; //placeholder
        public static final double ELEVATOR_SPEED = 0.5; //placeholder
    }

    public final class LedConstants
    {
        public static final int LED_Port = 4;
        public static final int LED_Length = 149; //length of led -1
    }






}
