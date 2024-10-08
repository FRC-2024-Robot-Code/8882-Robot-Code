package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Collect extends Command {

     Intake intake;
     Shooter shooter;

     public Collect(Intake intake, Shooter shooter) {
          this.intake = intake;
          this.shooter = shooter;

          addRequirements(intake, shooter);
     }

     @Override
     public void execute() {
          shooter.collectConveyor(0.23);
          intake.collectIntake();
     }

     @Override
     public void end(boolean interrupted) {
          shooter.stopConveyor();
          intake.stopIntake();
          this.cancel();
     }

     @Override
     public boolean isFinished() {
          return shooter.getIrState();
     }

}
