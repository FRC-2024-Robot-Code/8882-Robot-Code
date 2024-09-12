package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
     Shooter shooter;

     public Shoot(Shooter shooter) {
          this.shooter = shooter;
          addRequirements(shooter);
     }

     @Override
     public void execute() {
          shooter.setShooterSpeed(0.6);
     }

     @Override
     public void end(boolean interrupted) {

     }

     @Override
     public boolean isFinished() {
          return true;
     }
}
