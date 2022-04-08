/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArcadeDriveClassic;
import frc.robot.commands.AutoCross;
import frc.robot.commands.AutoFlywheelVelocityRun;
import frc.robot.commands.AutoReturn;
import frc.robot.commands.AutoStop;
import frc.robot.commands.AutoTurretAutoAlign;
import frc.robot.commands.BallShoot;
import frc.robot.commands.Elevator;
import frc.robot.commands.ElevatorNeutral;
import frc.robot.commands.ElevatorReverse;
import frc.robot.commands.FastElevator;
import frc.robot.commands.FlyWheelVelocityRunLow;
import frc.robot.commands.FlyWheelVelocityRunReverse;
import frc.robot.commands.Intake;
import frc.robot.commands.IntakeDrop;
import frc.robot.commands.IntakeIdle;
import frc.robot.commands.IntakeReverse;
import frc.robot.commands.IntakeStop;
import frc.robot.commands.IntakeUp;
import frc.robot.commands.NullCommand;
import frc.robot.commands.Queue;
import frc.robot.commands.TurretAutoAlign;
import frc.robot.commands.TurretNeutral;
import frc.robot.commands.TurretTurnLeft;
import frc.robot.commands.TurretTurnLeftSlow;
import frc.robot.commands.TurretTurnRight;
import frc.robot.commands.TurretTurnRightSlow;
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
  private FastElevator autoShoot = new FastElevator(elevatorsub);
  private Elevator autoShoot1 = new Elevator(elevatorsub);
  private Elevator autoShoot2 = new Elevator(elevatorsub);
  private AutoTurretAutoAlign autoTurretAutoAlign = new AutoTurretAutoAlign();
  private TurretAutoAlign TurretAutoAlign = new TurretAutoAlign();
  private Intake intake = new Intake(intakeSub);
  private WaitCommand waitCommand = new WaitCommand(2);
  private NullCommand nullCommand = new NullCommand();
  private NullCommand nullCommand2 = new NullCommand();
  private NullCommand nullCommand3 = new NullCommand();
  private AutoFlywheelVelocityRun flywheelVelocityRunTwoBall = new AutoFlywheelVelocityRun(flyWheel_Velocity, 8814);
  private AutoFlywheelVelocityRun flywheelVelocityRunFourBall = new AutoFlywheelVelocityRun(flyWheel_Velocity, 7021);
  private AutoFlywheelVelocityRun flywheelVelocityRunFiveBall = new AutoFlywheelVelocityRun(flyWheel_Velocity, 8814);
  private IntakeDrop autoDrop = new IntakeDrop(intakeSub);
  private AutoCross autoCross;
  private AutoReturn autoReturn;
 
  // declaring and intializing controller(s)
  public static XboxController xbox = new XboxController(OIConstants.XBOX_ID);
  public static XboxController helms = new XboxController(OIConstants.HELMS_ID);
  public static Joystick box = new Joystick(OIConstants.BOX_ID);
    final JoystickButton box1 = new JoystickButton(box, 1);


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
        new JoystickButton(helms, Button.kX.value).whenReleased(new IntakeIdle(intakeSub));
        new JoystickButton(xbox, Button.kY.value).whenReleased(new IntakeIdle(intakeSub));
        
      
       

    //turret stuff
        //new JoystickButton(helms, Button.kB.value).whenHeld(new TurretAutoAlign(turretSubsystem));
        new JoystickButton(helms, Button.kLeftStick.value).whenHeld(new TurretTurnLeftSlow(turretSubsystem));
        new JoystickButton(helms, Button.kRightStick.value).whenHeld(new TurretTurnRightSlow(turretSubsystem));
        new JoystickButton(helms, Button.kLeftBumper.value).whenHeld(new TurretTurnLeft(turretSubsystem));
        new JoystickButton(helms, Button.kRightBumper.value).whenHeld(new TurretTurnRight(turretSubsystem));
        //new JoystickButton(helms, Button.kA.value).whenHeld(new TurretCenter());
    //neutralizes turret
        new JoystickButton(helms, Button.kB.value).whenReleased(new TurretNeutral(turretSubsystem));
        new JoystickButton(helms, Button.kLeftStick.value).whenReleased(new TurretNeutral(turretSubsystem));
        new JoystickButton(helms, Button.kRightStick.value).whenReleased(new TurretNeutral(turretSubsystem));
        new JoystickButton(helms, Button.kLeftBumper.value).whenReleased(new TurretNeutral(turretSubsystem));
        new JoystickButton(helms, Button.kRightBumper.value).whenReleased(new TurretNeutral(turretSubsystem));

 
        new JoystickButton(box, 1).whenHeld(new IntakeDrop(new IntakeSub()));
        new JoystickButton(box, 2).whenHeld(new IntakeUp());
       
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

  public Joystick getBox() {
    return box;
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * currently there is no auto command
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    var table = NetworkTableInstance.getDefault().getTable("troubleshooting");
    var leftReference = table.getEntry("left_reference");
    var leftMeasurement = table.getEntry("left_measurement");
    var rightReference = table.getEntry("right_reference");
    var rightMeasurement = table.getEntry("right_measurement");

    var leftController = new PIDController(DriveConstants.Drive_Kp, 0, 0);
    var rightController = new PIDController(DriveConstants.Drive_Kp, 0, 0);

    Trajectory trajectoryOne = new Trajectory();
    Trajectory trajectoryTwo = new Trajectory();
    Trajectory trajectoryThree = new Trajectory();
    Trajectory trajectoryFour= new Trajectory();

    String trajectoryFileOne = "pathplanner/generatedJSON/2cargoreal.wpilib.json";
    //String trajectoryFileOne = "pathplanner/paths/3 carg.wpilib.json";
    String trajectoryFileTwo = "pathplanner/generatedJSON/4cargoreal.wpilib.json";
    String trajectoryFileThree = "pathplanner/generatedJSON/4cargoreverse.wpilib.json";
    String trajectoryFileFour = "pathplanner/generatedJSON/5cargoreal.wpilib.json";
   

    
    try{
      Path trajectoryPathOne = Filesystem.getDeployDirectory().toPath().resolve(trajectoryFileOne);
      trajectoryOne = TrajectoryUtil.fromPathweaverJson(trajectoryPathOne);
    } catch(IOException ex) {
        DriverStation.reportError("Unable to open trajectory:" + trajectoryFileOne, ex.getStackTrace());
    }

    try{
      Path trajectoryPathTwo = Filesystem.getDeployDirectory().toPath().resolve(trajectoryFileTwo);
      trajectoryTwo = TrajectoryUtil.fromPathweaverJson(trajectoryPathTwo);
    } catch(IOException ex) {
        DriverStation.reportError("Unable to open trajectory:" + trajectoryFileTwo, ex.getStackTrace());
    }

    try{
      Path trajectoryPathThree = Filesystem.getDeployDirectory().toPath().resolve(trajectoryFileThree);
      trajectoryThree = TrajectoryUtil.fromPathweaverJson(trajectoryPathThree);
    } catch(IOException ex) {
        DriverStation.reportError("Unable to open trajectory:" + trajectoryFileThree, ex.getStackTrace());
    }

    try{
      Path trajectoryPathFour = Filesystem.getDeployDirectory().toPath().resolve(trajectoryFileFour);
      trajectoryFour = TrajectoryUtil.fromPathweaverJson(trajectoryPathFour);
    } catch(IOException ex) {
        DriverStation.reportError("Unable to open trajectory:" + trajectoryFileFour, ex.getStackTrace());
    }
    
 
    
 

    //Call concatTraj in Ramsete to run both trajectories back to back as one
    //var concatTraj = trajectoryTwo.concatenate(trajectoryThree);


            RamseteCommand ramseteCommandOne =
            new RamseteCommand(
                trajectoryOne,
                driveSub::getPose,
                new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
                new SimpleMotorFeedforward(
                    DriveConstants.Drive_Ks,
                    DriveConstants.Drive_Kv,
                    DriveConstants.Drive_Ka),
                DriveConstants.kDriveKinematics,
                driveSub::getWheelSpeeds,
                leftController,
                rightController,
                // RamseteCommand passes volts to the callback
                driveSub::tankDriveVolts,
                driveSub);

              RamseteCommand ramseteCommandTwo =
                new RamseteCommand(
                    trajectoryTwo,
                    driveSub::getPose,
                    new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
                    new SimpleMotorFeedforward(
                        DriveConstants.Drive_Ks,
                        DriveConstants.Drive_Kv,
                        DriveConstants.Drive_Ka),
                    DriveConstants.kDriveKinematics,
                    driveSub::getWheelSpeeds,
                    leftController,
                    rightController,
                    // RamseteCommand passes volts to the callback
                    driveSub::tankDriveVolts,
                    driveSub);

                    RamseteCommand ramseteCommandTwoTwo =
                    new RamseteCommand(
                        trajectoryThree,
                        driveSub::getPose,
                        new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
                        new SimpleMotorFeedforward(
                            DriveConstants.Drive_Ks,
                            DriveConstants.Drive_Kv,
                            DriveConstants.Drive_Ka),
                        DriveConstants.kDriveKinematics,
                        driveSub::getWheelSpeeds,
                        leftController,
                        rightController,
                        // RamseteCommand passes volts to the callback
                        driveSub::tankDriveVolts,
                        driveSub);      
                    
              RamseteCommand ramseteCommandThree =
              new RamseteCommand(
                  trajectoryFour,
                  driveSub::getPose,
                  new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
                  new SimpleMotorFeedforward(
                      DriveConstants.Drive_Ks,
                      DriveConstants.Drive_Kv,
                      DriveConstants.Drive_Ka),
                  DriveConstants.kDriveKinematics,
                  driveSub::getWheelSpeeds,
                  leftController,
                  rightController,
                  // RamseteCommand passes volts to the callback
                  driveSub::tankDriveVolts,
                  driveSub);


    leftMeasurement.setNumber(driveSub.getWheelSpeeds().leftMetersPerSecond);
    leftReference.setNumber(leftController.getSetpoint());

    rightMeasurement.setNumber(driveSub.getWheelSpeeds().rightMetersPerSecond);
    rightReference.setNumber(rightController.getSetpoint());
    
    // Reset odometry to the starting pose of the trajectory.
    driveSub.resetOdometry(trajectoryOne.getInitialPose());

    // Run path following command, then stop at the end.
    //return ramseteCommand.andThen(() -> driveSub.tankDriveVolts(0, 0));
    return new ParallelCommandGroup(new SequentialCommandGroup(/*autoShoot1.withTimeout(.5),*/ flywheelVelocityRunTwoBall,
     ramseteCommandOne.andThen(() -> driveSub.tankDriveVolts(0, 0)), nullCommand2.withTimeout(.1), autoShoot.withTimeout(.9), flywheelVelocityRunFourBall,
     ramseteCommandTwo.andThen(() -> driveSub.tankDriveVolts(0, 0)), nullCommand.withTimeout(.1), ramseteCommandTwoTwo.andThen(() -> driveSub.tankDriveVolts(0, 0)), autoShoot1.withTimeout(1), 
     flywheelVelocityRunFiveBall, ramseteCommandThree.andThen(() -> driveSub.tankDriveVolts(0, 0)), autoShoot2), autoTurretAutoAlign, autoDrop.withTimeout(.5));
   

    //return new ParallelCommandGroup(new SequentialCommandGroup(flyWheelVelocityRunTwoBall, ramseteCommandOne.andThen(() -> driveSub.tankDriveVolts(0, 0)), autoShoot), autoTurretAutoAlign, autoDrop.withTimeout(.5)); 
    }   
  }

