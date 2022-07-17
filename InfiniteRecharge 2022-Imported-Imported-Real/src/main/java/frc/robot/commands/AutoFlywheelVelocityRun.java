// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlyWheel_Velocity;
import frc.robot.utils.Limelight;
import frc.robot.utils.Limelight.LightMode;


public class AutoFlywheelVelocityRun extends CommandBase {
  /** Creates a new FlyWheelVelocityRun. */
  double speedToGet; //instantiates variable speedToGet
  private final FlyWheel_Velocity m_flyWheel_Velocity;
  public AutoFlywheelVelocityRun(FlyWheel_Velocity f, double speed)  { //requires flywheel_velocity subsystem and a double speed as input
  
     m_flyWheel_Velocity = f;

      speedToGet = speed;     //sets speed to get as the speed input given to the command
    
        // Use addRequirements() here to declare subsystem dependencies.
        //THIS WAS CODED BADLY. IT SHOULD REQUIRE THE FLYWHEEL SUBSYSTEM. DO NOT COPY THIS COMMAND WITHOUT ADDING THAT
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_flyWheel_Velocity.my_Flywheel_Velocity(speedToGet); // sets flywheel velocity to speedToGet
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Limelight.setLedMode(LightMode.eOff); //turns of limelight
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true; 
  }
}
