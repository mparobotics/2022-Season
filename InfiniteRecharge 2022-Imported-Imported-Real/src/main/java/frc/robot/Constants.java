// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final static class DriveConstants{
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
        public static final double Drive_Ks = .60713;
        public static final double Drive_Kv = .18424;
        public static final double Drive_Ka = .036145;
        public static final double Drive_Kp = 1.4948;
        public static final double Drive_KpTest = 0;
        public static final double Drive_Kd = 0.0;
        

        public static final double DRIVE_P = 4; /**not sure what these do TODO ask mikey but im pretty sure theyre the offset
         for the drive forward but yeah so like do we need to change this Mikey? If you see this remind me to ask them, it was 
         not documented in Annie's code*/
        public static final double DRIVE_I = 1;

        public static final double TIC_FT = ((Math.PI)/2048)/10.91; //inches?
        public static final double kTrackwidthMeters = 0.73253;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        public static final double GearRatio = 10.91;
        public static final int EncoderTPR = 2048;
        public static final double WheelDiameterMeters = Units.inchesToMeters(6.0);
        public static final double WheelRadiousMeters  = Units.inchesToMeters(3.0);
        public static final double EncoderDistancePerPulse =
            // Uses the integrated Falcon 500 Encoders
            (WheelDiameterMeters * Math.PI)/ GearRatio / (double) EncoderTPR;
        public static final double WheelCircumferenceMeters = 2*Math.PI*WheelRadiousMeters;
        public static final double Conversion = WheelCircumferenceMeters/(GearRatio*EncoderTPR);

        // ((sensor/2048)/10.91)*2*pi*(2inch)
        // ((sensor/2048)/10.91)*WheelCircumferenceMeters
        
        public static final double MPArunitsToMeters = 30.30303;

    }

    public final class ShooterConstants
    {
        public static final int NEO_TURRET_ID = 51;
        public static final float Turret_Kp = -0.005f; //speed modifier of le turret
        public static final float min_command = 0.00001f; //deadzone of le turret
        public static final double hub_height = 2.74 ; //meters 
        public static final double limelight_height = 1.21; //metres
        public static final double limelight_angle = 21; //degrees
        public static final int FALCON_shooter_ID = 45; 
        public static final double kP = .019117; //proportional feedback constant 12.087
        public static final double kA = .42682; // acceleration constant of shooter
        public static final double kV = .39505; // describes how much voltage is needed to hold (or “cruise”) at a given constant velocity while overcoming the electromagnetic resistance in the motor and any additional friction that increases with speed (known as viscous drag). The relationship between speed and voltage (at constant acceleration) is almost entirely linear
        public static final double kS = 0.73835; // voltage needed to overcome the motor’s static friction
        public static final double kD = 0;
        public static final int hood_encoder_ratio = 0; //ratio of encoder count to degree angle of turret, placeholder
        //public static final int hood_motor_ID = 52; 
        public static final double hood_min_command = 7;
        public static final double max_turret_rotation = 10000; //placeholder
        public static final double centering_speed = .3;

        
        public static final int LIMELIGHT_SERVO_ID = 3;

        public static final double HIGH_GOAL_SPEED = .6; //Changed
        public static final int LIMELIGHT_ANGLE_SETPOINT = 57 + 90; //far shoot
        public static final int LIMELIGHT_ANGLE_SETPOINT2 = 10 + 90; //key shoot

        public static final double SHOOTER_P = 1;
        public static final double SHOOTER_I = 1;
        public static final double SHOOTER_SETPOINT = 4400; 
        public static final double SHOOTER_MAX_VELOCITY = 5676;

        public static final double AUTO_SHOOT_TIME = 5;
    }

    public final class OIConstants {
        public static final int XBOX_ID = 0;
        public static final int HELMS_ID = 1;
    
    }

    public final class IntakeConstants
    {
        public static final int INTAKE_ID = 15; 
        public static final double INTAKE_SPEED = 1; //placeholder
        public static final int INTAKE_DROPDOWN_ID = 53; 
        public static final double DROPDOWN_SPEED = -.3; //placeholder
    }

    public final class ElevatorConstants
    {
        public static final int FRONT_ELEVATOR_ID = 32;
        public static final int BACK_ELEVATOR_ID = 26; 
        
        public static final double ELEVATOR_SPEED = 1; //placeholder
       
    
        public static final int LINEBREAK_RECIVER_ID = 10;
    }

    public final class LedConstants
    {
        public static final int LED_Port = 4;
        public static final int LED_Length = 149; //length of led -1
    }






}
