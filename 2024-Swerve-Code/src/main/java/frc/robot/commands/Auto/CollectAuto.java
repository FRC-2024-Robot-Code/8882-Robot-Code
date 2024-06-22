package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AngleShooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class CollectAuto extends Command {
     Intake intake;
     Shooter shooter;
     AngleShooter angle;

     public CollectAuto(Intake intake, Shooter shooter, AngleShooter angle) {
          this.intake = intake;
          this.shooter = shooter;
          this.angle = angle;
          addRequirements(intake, shooter, angle);
     }

     @Override
     public void initialize() {
          intake.collect();
          shooter.collect();
          angle.setTarget(0.54);
     }

     @Override
     public boolean isFinished() {
          return shooter.getState();
     }

     @Override
     public void end(boolean interrupted) {
          shooter.stopMotorConveyor();
          intake.stop();
     }
}
