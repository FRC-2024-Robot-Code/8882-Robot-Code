package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Angle extends SubsystemBase {
     CANSparkMax angle1, angle2;
     LimelightHelpers camera = new LimelightHelpers();
     InterpolatingDoubleTreeMap interpolating = new InterpolatingDoubleTreeMap();
     DutyCycleEncoder angleDuty;
     PIDController anglePID;

     public Angle() {
          angle1 = new CANSparkMax(11, MotorType.kBrushless);
          angle2 = new CANSparkMax(12, MotorType.kBrushless);

          angle1.setInverted(true);
          angle2.setInverted(false);

          angleDuty = new DutyCycleEncoder(4);

          angleDuty.setDutyCycleRange(0.15, 0.75);
          anglePID = new PIDController(1.3, 0, 0);
     }

     public void stopAngle() {
          angle1.stopMotor();
          angle2.stopMotor();
     }

     public void setAngleSpeed(double output) {
          angle1.set(output);
          angle2.set(output);
     }

     public double getTz() {

          if (LimelightHelpers.getTargetCount("") == 0) {
               return 2.1;
          }
          double[] get = LimelightHelpers.getTargetPose_RobotSpace("");
          return get[2];
     }

     public double getTx() {
          return LimelightHelpers.getTX("");
     }

     public double getAngle() {
          double inter = 0;
          double[] get = LimelightHelpers.getTargetPose_RobotSpace("");
          if (get.length >= 2) {
               double tz = get[2];
               inter = interpolating.get(tz);
               if (inter > 1) {
                    inter = 0.8;
               } else if (inter < 0.1) {
                    inter = 0.5;
               }
          }
          return inter;
     }

     public double getABSgyro() {
          return angleDuty.getAbsolutePosition();
     }

     @Override
     public void periodic() {

          SmartDashboard.putNumber("Angle", getABSgyro());
     }
}
