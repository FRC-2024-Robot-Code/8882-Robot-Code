package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AngleShooter;
import frc.robot.subsystems.Shooter;

public class ShootAmp extends SequentialCommandGroup {

  public ShootAmp(Shooter shooter, AngleShooter angle) {
    addCommands(
        Commands.runOnce(() -> angle.setTarget(0.985), angle),
        Commands.runOnce(() -> shooter.setSpeed(0.18), shooter),
        new WaitCommand(1),
        Commands.runOnce(() -> shooter.setSpeedConveyor(0.3), shooter),
        new WaitCommand(1),
        Commands.runOnce(() -> shooter.stopAll(), shooter),
        Commands.runOnce(() -> this.cancel()));
  }
}
