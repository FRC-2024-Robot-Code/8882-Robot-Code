package frc.robot.commands.Auto.AmpAuto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter;

public class aShoot2 extends SequentialCommandGroup {
     public aShoot2(Shooter shooter) {
          addCommands(
                    Commands.runOnce(() -> shooter.setSpeed(0.65), shooter),
                    new WaitCommand(3),
                    Commands.runOnce(() -> shooter.setSpeedConveyor(0.8), shooter),
                    new WaitCommand(2),
                    Commands.runOnce(() -> shooter.stopAll(), shooter),
                    Commands.runOnce(() -> this.cancel())

          );
     }
}
