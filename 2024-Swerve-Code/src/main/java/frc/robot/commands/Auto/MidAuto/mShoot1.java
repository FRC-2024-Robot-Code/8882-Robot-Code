package frc.robot.commands.Auto.MidAuto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AngleShooter;
import frc.robot.subsystems.Shooter;

public class mShoot1 extends SequentialCommandGroup {
     public mShoot1(Shooter shooter, AngleShooter angle) {
          addCommands(
                    Commands.runOnce(() -> shooter.setSpeed(0.50), shooter),
                    new WaitCommand(1.2),
                    Commands.runOnce(() -> shooter.setSpeedConveyor(0.8), shooter),
                    new WaitCommand(0.7),
                    Commands.runOnce(() -> shooter.stopAll(), shooter),
                    Commands.runOnce(() -> this.cancel()));
     }
}
