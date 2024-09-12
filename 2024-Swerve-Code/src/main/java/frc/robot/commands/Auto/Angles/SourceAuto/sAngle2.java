package frc.robot.commands.Auto.Angles.SourceAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angle;

public class sAngle2 extends Command {
     Angle angle;
     double setpoint;
     PIDController pid = new PIDController(2, 0, 0);

     public sAngle2(Angle angle) {

          this.angle = angle;
          addRequirements(angle);
     }

     @Override
     public void initialize() {
     }

     @Override
     public void execute() {

          double output = pid.calculate(angle.getABSgyro(), 0.21);
          output = MathUtil.clamp(output, -3, 3);
          angle.setAngleSpeed(output);
     }

     @Override
     public void end(boolean interrupted) {
          angle.stopAngle();
          this.cancel();
     }

     @Override
     public boolean isFinished() {
          return pid.atSetpoint();

     }
}
