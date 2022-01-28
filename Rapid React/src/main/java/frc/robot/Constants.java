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
        public static final int FALCON_FR_ID = 0;
        public static final int FALCON_FL_ID = 0;
        public static final int FALCON_BR_ID = 0;
        public static final int FALCON_BL_ID = 0;
        

        public static final double DRIVE_P = 4; /**not sure what these do TODO ask mikey but im pretty sure theyre the offset
         for the drive forward but yeah so like do we need to change this Mikey? If you see this remind me to ask them, it was 
         not documented in Annie's code*/
        public static final double DRIVE_I = 1;

        //public static final double TIC_FT = ((Math.PI)/2014)/10.75; (Is this accurate still?)


    }

    public final class ShooterConstants
    {
        public static final int NEO_TURRET_ID = 0;
        public static final float Kp = 0.033f; //to fix 
        public static final float min_command = 0.033f; //to fix?
        public static final double hub_height = 104; //inches
        public static final double limelight_height = 8.875; //inches
        public static final double limelight_angle = 45; //degrees

    }

    public final class OIConstants {
        public static final int XBOX_ID = 0;
        public static final int HELMS_ID = 1;
    
    }







}
