// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public DriveSubsystem drivesub = new DriveSubsystem();

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

    //Create config for Trajectory
    TrajectoryConfig config =
      new TrajectoryConfig(
        DriveConstants.kMaxSpeedMetersPerSecond,
        DriveConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

    //TODO ADD TRAJECTORY IMPORT

    RamseteCommand ramseteCommand =
      new RamseteCommand(
        trajectory, 
        pose, 
        controller, 
        feedforward, 
        kinematics, 
        wheelSpeeds, 
        leftController, 
        rightController, 
        outputVolts, 
        requirements);

    //Reset Odometry to the starting pose of the trajectory
    driveSub.resetOdometry(trajectory.getInitialPose());

    //Run path following command, then stop at the end
    return ramseteCommand.andThen(() -> driveSub.tankDriveVolts(0, 0));
  }
}
