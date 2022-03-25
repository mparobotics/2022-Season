// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public DriveSubsystem driveSub = new DriveSubsystem();

  public static XboxController xbox = new XboxController(OIConstants.XBOX_ID);
  public static XboxController helms = new XboxController(OIConstants.HELMS_ID);
  public static Joystick shooterStick = new Joystick(2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

    public XboxController getController() {
      return xbox;
    }

    public XboxController getHelms() {
      return helms;
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //creating a voltage constraint to ensure we don't accelerate to fast
    var autoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
          DriveConstants.Drive_Ks, //kVolts
          DriveConstants.Drive_Kv, //kVoltSecondsPerMeter
          DriveConstants.Drive_Ka), //kVoltsSecondsSquaredPerMeter
        DriveConstants.kDriveKinematics,
        10);

    String trajectoryJSON = "paths/Testing.wpilib.json";
    Trajectory trajectory = new Trajectory();

    try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    //Create config for Trajectory
    TrajectoryConfig config = //TODO FIX
      new TrajectoryConfig(
        DriveConstants.kMaxSpeedMetersPerSecond,
        DriveConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

    RamseteCommand ramseteCommand =
      new RamseteCommand(
        trajectory, //Load Trajectory being followed 
        driveSub::getPose, //Supplies Robot Pose
        new RamseteController( //THis performs the path-following computation that translates the current measured pose and Trajectorys tate into a chassis speed setpoint
          DriveConstants.kRamseteB,
          DriveConstants.kRamseteZeta), 
        new SimpleMotorFeedforward( //Automatically performs the correct feedforward calculation with the feedforward gains kV and kA from (Sysid)
          DriveConstants.Drive_Ks,
          DriveConstants.Drive_Kv,
          DriveConstants.Drive_Ka), 
        DriveConstants.kDriveKinematics, //Pulls Kinematics from DriveConstants. (Convert Chassis speeds to wheel speeds)
        driveSub::getWheelSpeeds, //Wheel speed supplier. References to the drive subsystem method that returns wheel speeds
        new PIDController(DriveConstants.Drive_Kp, 0, 0), //Left-side PIDController. Will Track the Left Side wheel speed setpoint using Pgain (SysID)
        new PIDController(DriveConstants.Drive_Kp, 0, 0), //Right-side PIDController. Will Track the Left Side wheel speed setpoint using Pgain (SysID)
        driveSub::tankDriveVolts,
        driveSub); //The drive subsystem. Included to ensure the command does not operate on the drive at the same time as any other command that use the drive

    //Reset Odometry to the starting pose of the trajectory
    driveSub.resetOdometry(trajectory.getInitialPose());

    //Run path following command, then stop at the end
    return ramseteCommand.andThen(() -> driveSub.tankDriveVolts(0, 0));
  }
}
