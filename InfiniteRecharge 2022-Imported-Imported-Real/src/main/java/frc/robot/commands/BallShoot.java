// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.FlyWheel_Velocity;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BallShoot extends ParallelCommandGroup {
  /** Creates a new BallShoot. */
  public BallShoot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new FlyWheelVelocityRun(new FlyWheel_Velocity()),new TurretAutoAlign());
    /*parralel command group hell! oh boy, this ones a doozy. I will now attempt to regurgitate
    * what was told to me at a week one regional when i was stressed, panicking, and overall not doing
    *too great. I will also give context for why we use these. Commands with dependancies for one
    *subsystem don't like pulling things from others. Lets say, as a *completly hypothetical* example
    * that *definitley* didn't happen to a certain coder, you wanted to both align the turret, and
    *spin up the flywheel under one command. Now, because you think that you are smart, you put to subs,
    *in one command. The next match (which your team payed 500$ to play in because regionals go brrr),
    *the entire robot is lagging, moving in a very choppy motion. Whatever could it be!? the robot was working fine last match
    *well, stupid little you decided to only add this in that match, and the robot kinda loop overuns and lags bad
    *when you do something like that. Well then, what would I do instead to get this command, oh so enlightened reader may ask.
    *INTRODUCING PARALLEL COMMAND GROUPS! as a helpful CSA who magically found your problem you didn't realize would be a problem,
    *so kindly points out, you can turn one command into two and run them at the same time in, *gasps* parallel!!! thats what this does
    *and why we use it
    *
    * the completly hypothetical match can be found at https://www.thebluealliance.com/match/2022mndu2_qm2
    */

  }
}
