package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AngleShooter;

public class Return0 extends Command {

     AngleShooter angle;

     public Return0(AngleShooter subsystem) {
          angle = subsystem;

          addRequirements(subsystem);
     }

     @Override
     public void initialize() {
     }

     @Override
     public void execute() {
          angle.setTarget(0.1);
     }

     @Override
     public void end(boolean interrupted) {
     }

     @Override
     public boolean isFinished() {
          return false;
     }
}