// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.FlyWheel_Velocity;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;


public class AutoShootBall extends CommandBase {

  private final FlyWheel_Velocity m_flywheelsub;
  /** Creates a new AutoShootBall. */
  public AutoShootBall(FlyWheel_Velocity fVelocity) {
    m_flywheelsub = fVelocity;
    addRequirements(m_flywheelsub);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double d = TurretSubsystem.getDistance();
    double speedToGet;
   // d = SmartDashboard.getNumber("distance sim", 3);



    FlyWheel_Velocity.my_Flywheel_Velocity(4500);
  }
 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  
    ElevatorSub.ElevateBall(ElevatorConstants.ELEVATOR_SPEED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  
    if (Timer.getMatchTime() < 3.0){
    return true;}
    else {return false;}
  }
}
