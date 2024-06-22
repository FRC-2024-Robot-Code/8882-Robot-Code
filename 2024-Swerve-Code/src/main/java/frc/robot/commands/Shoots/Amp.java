package frc.robot.commands.Shoots;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter;

public class Amp extends SequentialCommandGroup {
     public Amp(Shooter shooter) {
          addCommands(
                    Commands.runOnce(() -> shooter.setShooterSpeed(0.2), shooter),
                    new WaitCommand(1),
                    Commands.runOnce(() -> shooter.setConveyorSpeed(0.3), shooter),
                    new WaitCommand(1),
                    Commands.runOnce(() -> {
                         shooter.stopShooter();
                         shooter.stopConveyor();
                    }, shooter),
                    Commands.runOnce(() -> this.cancel()));
     }
}
