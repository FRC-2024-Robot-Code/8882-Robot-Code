package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class CollectAuto extends Command {

     Intake intake;
     Shooter shooter;
     Timer timer = new Timer();

     public CollectAuto(Intake intake, Shooter shooter) {
          this.intake = intake;
          this.shooter = shooter;
          addRequirements(intake, shooter);
     }

     @Override
     public void initialize() {
          // timer.start();
     }

     @Override
     public void execute() {
          shooter.collectConveyor(0.19);
          intake.collectIntake();
     }

     @Override
     public void end(boolean interrupted) {
          shooter.stopConveyor();
          intake.stopIntake();
          // timer.reset();
          this.cancel();
     }

     @Override
     public boolean isFinished() {
          // if (timer.get() > 4) {
          //      return true;
          // }
          return shooter.getIrState();
          // return true;
     }

}
