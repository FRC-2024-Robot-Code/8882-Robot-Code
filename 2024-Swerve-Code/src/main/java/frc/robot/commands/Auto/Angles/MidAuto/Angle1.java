package frc.robot.commands.Auto.Angles.MidAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angle;

public class Angle1 extends Command {
     Angle robot;
     PIDController pidController;
     double setpoint;

     public Angle1(Angle robot) {

          this.robot = robot;
          pidController = new PIDController(3, 0, 0);

          addRequirements(robot);
     }

     @Override
     public void initialize() {
     }

     @Override
     public void execute() {
          double outPut = pidController.calculate(robot.getABSgyro(), 0.29);

          outPut = MathUtil.clamp(outPut, -0.4, 0.4);

          robot.setAngleSpeed(outPut);
     }

     @Override
     public void end(boolean interrupted) {
          robot.stopAngle();
          this.cancel();
     }

     @Override
     public boolean isFinished() {
          return pidController.atSetpoint();
     }
}
