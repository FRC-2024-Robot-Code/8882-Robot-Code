package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Collect extends Command {

     Shooter shooter;
     Intake intake;

     public Collect(Shooter sub1, Intake sub2) {
          shooter = sub1;
          intake = sub2;

          addRequirements(sub1, sub2);
     }

     @Override
     public void initialize() {
     }

     @Override
     public void execute() {
          shooter.collect();
          intake.collect();
     }

     @Override
     public void end(boolean interrupted) {
     }

     @Override
     public boolean isFinished() {
          return false;
     }
}
