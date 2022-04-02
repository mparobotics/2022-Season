/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArcadeDriveClassic;
import frc.robot.commands.AutoCross;
import frc.robot.commands.AutoReturn;
import frc.robot.commands.AutoStop;
import frc.robot.commands.BallShoot;
import frc.robot.commands.Elevator;
import frc.robot.commands.ElevatorNeutral;
import frc.robot.commands.ElevatorReverse;
import frc.robot.commands.FlyWheelVelocityRunLow;
import frc.robot.commands.FlyWheelVelocityRunReverse;
import frc.robot.commands.Intake;
import frc.robot.commands.IntakeDrop;
import frc.robot.commands.IntakeIdle;
import frc.robot.commands.IntakeReverse;
import frc.robot.commands.IntakeStop;
import frc.robot.commands.IntakeUp;
import frc.robot.commands.Queue;
import frc.robot.commands.TurretNeutral;
import frc.robot.commands.TurretTurnLeft;
import frc.robot.commands.TurretTurnRight;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.FlyWheel_Velocity;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.TurretSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 * 
 * Contains subsystems, OI devices, and commands (runs commands)
 */
public class RobotContainer {
  // declaring and intializing subsystems(all) and commands (most commands don't need to be delclared/intialized here)
  public DriveSubsystem driveSub      = new DriveSubsystem();
  public IntakeSub intakeSub = new IntakeSub(); 
  public ElevatorSub elevatorsub = new ElevatorSub();
  //public static ShooterSubsystem shooterSub = new ShooterSubsystem();
  public static FlyWheel_Velocity flyWheel_Velocity = new FlyWheel_Velocity();
  public static TurretSubsystem turretSubsystem = new TurretSubsystem();
  private Elevator autoShoot = new Elevator(elevatorsub);
  private Elevator autoShoot1 = new Elevator(elevatorsub);
  private Elevator autoShoot2 = new Elevator(elevatorsub);
  private AutoStop autoStop = new AutoStop(new DriveSubsystem());
  private AutoStop autoStop1 = new AutoStop(new DriveSubsystem());
  private AutoStop autoStop2 = new AutoStop(new DriveSubsystem());
  private WaitCommand waitCommand = new WaitCommand(2);
  private IntakeDrop autoDrop = new IntakeDrop(intakeSub);
  private AutoCross autoCross;
  private AutoReturn autoReturn;
 
  // declaring and intializing controller(s)
  public static XboxController xbox = new XboxController(OIConstants.XBOX_ID);
  public static XboxController helms = new XboxController(OIConstants.HELMS_ID);
  public static Joystick shooterStick = new Joystick(2);

  //private Joystick buttonBoard = new Joystick(OIConstants.buttonBoard); //TODO ButtonBoardTEST

  public static boolean shooting = false;
   
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    //drive controls
    /*driveSub.setDefaultCommand(new ArcadeDriveTrigger(driveSub, 
                              () -> xbox.getLeftTriggerAxis(), 
                              () -> xbox.getRightTriggerAxis(),
                              () -> xbox.getLeftX()));*/
  
    driveSub.setDefaultCommand(new ArcadeDriveClassic(driveSub,
                               () -> xbox.getLeftY(),
                               () -> xbox.getRightX()*.7));
                        
    // Configure the button bindings
    //shooterSub.setDefaultCommand(new ShootBall(shooterSub, shooterStick.getY()));
    intakeSub.setDefaultCommand(new IntakeIdle(intakeSub));
    
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //Main's Commands - Winch is right stick, 
    //new JoystickButton(xbox, Button.kBumperRight.value).whenHeld(new StartWinch(climberSub)); // Disabled - Climber Taken off
      //new JoystickButton(helms, Button.kB.value).whenHeld(new ShootBall(shooterSub, 2));
      new JoystickButton(helms, Button.kB.value).whenHeld(new BallShoot());
      new JoystickButton(helms, Button.kStart.value).whenHeld(new FlyWheelVelocityRunLow(flyWheel_Velocity));
      //new JoystickButton(helms, Button.kY.value).whenHeld(new TurretAutoAlign(turretSubsystem));
      //new JoystickButton(helms, Button.kY.value).whenHeld(new ShootBall(shooterSub, 2));
      //new JoystickButton(helms, Button.kRightStick.value).whenHeld(new ShootLow(shooterSub));
      //new JoystickButton(helms, Button.kLeftStick.value).whenHeld(new ShootReverse(shooterSub));
 
      
    //intake settings
        new JoystickButton(helms, Button.kX.value).whenHeld(new Intake(intakeSub));
        new JoystickButton(helms, Button.kY.value).whenHeld(new IntakeReverse(intakeSub));
    //intake stop
        new JoystickButton(helms, Button.kX.value).whenReleased(new IntakeStop());
        new JoystickButton(xbox, Button.kY.value).whenReleased(new IntakeStop());
        
      
       

    //turret stuff
        //new JoystickButton(helms, Button.kB.value).whenHeld(new TurretAutoAlign(turretSubsystem));
        new JoystickButton(helms, Button.kLeftBumper.value).whenHeld(new TurretTurnLeft(turretSubsystem));
        new JoystickButton(helms, Button.kRightBumper.value).whenHeld(new TurretTurnRight(turretSubsystem));
        //new JoystickButton(helms, Button.kA.value).whenHeld(new TurretCenter());
    //neutralizes turret
        new JoystickButton(helms, Button.kB.value).whenReleased(new TurretNeutral(turretSubsystem));
        new JoystickButton(helms, Button.kLeftBumper.value).whenReleased(new TurretNeutral(turretSubsystem));
        new JoystickButton(helms, Button.kRightBumper.value).whenReleased(new TurretNeutral(turretSubsystem));

 
        new JoystickButton(helms, Button.kRightStick.value).whenHeld(new IntakeDrop(new IntakeSub()));
        new JoystickButton(helms, Button.kLeftStick.value).whenHeld(new IntakeUp());
       
    //intake dropdown
        //new JoystickButton(helms, Button.kX.value).whenHeld(new IntakeDrop());
       
        
    //Elevator settings
        //new JoystickButton(helms, Button.kA.value).whenHeld(new Queue(elevatorsub));
        new JoystickButton(helms, Button.kA.value).whenHeld(new Elevator(elevatorsub));
        new JoystickButton(helms, Button.kY.value).whenHeld(new ElevatorReverse(elevatorsub));
        
        new JoystickButton(helms, Button.kY.value).whenHeld(new FlyWheelVelocityRunReverse(flyWheel_Velocity));
        //elevator stop
        new JoystickButton(helms, Button.kB.value).whenReleased(new ElevatorNeutral(elevatorsub));
        new JoystickButton(helms, Button.kY.value).whenReleased(new ElevatorNeutral(elevatorsub));
    


    //Helm's Commands
    //new JoystickButton(helms, Button.kBack.value).whenHeld(new ClimberUp(climberSub)); // Disabled - Climber Taken off
  }

  public XboxController getController() {
    return xbox; 
  }

  public XboxController getHelms() {
    return helms;
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * currently there is no auto command
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //Create a voltage constraint to ensure we don't accelerate too fast

    autoCross = new AutoCross(driveSub);
    autoReturn = new AutoReturn(driveSub);

    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveConstants.Drive_Ks,
                DriveConstants.Drive_Kv,
                DriveConstants.Drive_Ka),
            DriveConstants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                DriveConstants.kMaxSpeedMetersPerSecond,
                DriveConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    //Trajectory trajectory = new Trajectory();

    var trajectory =
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 3, new Rotation2d(0)), 
        List.of(new Translation2d(1,3), new Translation2d(2, 3)),
        new Pose2d(3, 3, new Rotation2d(0)), config);

    //String myPathName = "";
    String trajectoryfile = "paths/Testing.wpilib.json";

    //myPathName = "Unamed";

    //trajectoryfile = myPathName + ".wpilib.json";
    try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryfile);
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryfile, ex.getStackTrace());
    }

    var trajectory1 =
    TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 3, new Rotation2d(0)), 
      List.of(new Translation2d(1,3), new Translation2d(2, 3)),
      new Pose2d(3, 3, new Rotation2d(0)), config);

  //String myPathName = "";
  String trajectoryfile1 = "paths/Testing.wpilib.json";

  //myPathName = "Unamed";

  //trajectoryfile = myPathName + ".wpilib.json";
  try {
      Path trajectoryPath1 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryfile1);
      trajectory1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath1);
  } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryfile, ex.getStackTrace());
  }

  trajectory1 = TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(0, 0, new Rotation2d(0)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(3, 0, new Rotation2d(0)),
    // Pass config
    config);
  


            RamseteCommand ramseteCommand =
            new RamseteCommand(
                trajectory,
                driveSub::getPose,
                new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
                new SimpleMotorFeedforward(
                    DriveConstants.Drive_Ks,
                    DriveConstants.Drive_Kv,
                    DriveConstants.Drive_Ka),
                DriveConstants.kDriveKinematics,
                driveSub::getWheelSpeeds,
                new PIDController(DriveConstants.Drive_Kp, 0, 0),
                new PIDController(DriveConstants.Drive_Kp, 0, 0),
                // RamseteCommand passes volts to the callback
                driveSub::tankDriveVolts,
                driveSub);

              RamseteCommand ramseteCommand1 =
                new RamseteCommand(
                    trajectory1,
                    driveSub::getPose,
                    new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
                    new SimpleMotorFeedforward(
                        DriveConstants.Drive_Ks,
                        DriveConstants.Drive_Kv,
                        DriveConstants.Drive_Ka),
                    DriveConstants.kDriveKinematics,
                    driveSub::getWheelSpeeds,
                    new PIDController(DriveConstants.Drive_Kp, 0, 0),
                    new PIDController(DriveConstants.Drive_Kp, 0, 0),
                    // RamseteCommand passes volts to the callback
                    driveSub::tankDriveVolts,
                    driveSub);
    
    // Reset odometry to the starting pose of the trajectory.
    driveSub.resetOdometry(trajectory1.getInitialPose());

    // Run path following command, then stop at the end.
    //return ramseteCommand.andThen(() -> driveSub.tankDriveVolts(0, 0));
    return new SequentialCommandGroup(ramseteCommand.andThen(() -> driveSub.tankDriveVolts(0, 0)),
                                       autoShoot, ramseteCommand1.andThen(()-> driveSub.tankDriveVolts(0, 0)), autoShoot1);
    }
  }


