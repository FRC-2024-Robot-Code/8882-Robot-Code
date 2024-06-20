package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AngleShooter;

public class Angle extends Command {

     AngleShooter angle;

     public Angle(AngleShooter subsystem) {
          angle = subsystem;

          addRequirements(subsystem);
     }

     @Override
     public void initialize() {
     }

     @Override
     public void execute() {
          angle.setTarget(angle.getAngle());
     }

     @Override
     public void end(boolean interrupted) {
     }

     @Override
     public boolean isFinished() {
          return false;
     }
}