package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angle;

public class AngleSpeaker extends Command {

     Angle angle;
     PIDController pidController;

     double setpoint;
     InterpolatingDoubleTreeMap anglesMap = new InterpolatingDoubleTreeMap();
     double inter = 0;

     public AngleSpeaker(Angle angle) {
          this.angle = angle;
          pidController = new PIDController(3, 0, 0);

          anglesMap.put(2.1, 0.3);
          anglesMap.put(2.2, 0.29); // 2.22
          anglesMap.put(2.3, 0.285); // 2.3
          anglesMap.put(2.4, 0.275); // 2.4
          anglesMap.put(2.5, 0.27); // 2.5
          anglesMap.put(2.6, 0.26); // 2.64
          anglesMap.put(2.8, 0.255); // 2.8
          anglesMap.put(2.9, 0.242); // 2.9
          anglesMap.put(3.0, 0.24); // 3.0
          anglesMap.put(3.1, 0.234); // 3.1
          anglesMap.put(3.2, 0.228); // 3.2
          anglesMap.put(3.3, 0.222); // 3.3

          addRequirements(angle);
     }

     @Override
     public void initialize() {
     }

     @Override
     public void execute() {
          inter = anglesMap.get(angle.getTz());
          if (inter > 1) {
               inter = 0.8;
          } else if (inter < 0.1) {
               inter = 0.5;
          }

          double outPut = pidController.calculate(angle.getABSgyro(), inter);
          outPut = MathUtil.clamp(outPut, -0.3, 0.3);
          angle.setAngleSpeed(outPut);
     }

     @Override
     public void end(boolean interrupted) {
          angle.stopAngle();
     }

     @Override
     public boolean isFinished() {
          return false;
     }
}
