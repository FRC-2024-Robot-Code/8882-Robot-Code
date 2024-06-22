package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Angle extends SubsystemBase {
     CANSparkMax angle1, angle2;
     RelativeEncoder angleEncoder;
     LimelightHelpers camera = new LimelightHelpers();
     InterpolatingDoubleTreeMap interpolating = new InterpolatingDoubleTreeMap();

     public Angle() {
          angle1 = new CANSparkMax(11, MotorType.kBrushless);
          angle2 = new CANSparkMax(12, MotorType.kBrushless);

          angleEncoder = angle2.getEncoder();

          angle1.setInverted(true);
          angle2.setInverted(false);

          interpolating.put(1.31, 0.8);
          interpolating.put(1.51, 0.5);
     }

     public void stopAngle() {
          angle1.stopMotor();
          angle2.stopMotor();
     }

     public void setAngleSpeed(double output) {
          angle1.set(output);
          angle2.set(output);
     }

     public double getAngleSpeed() {
          return angleEncoder.getVelocity();
     }

     public void resetAngleEncoder() {
          angleEncoder.setPosition(0);
     }

     public double getAnglePosition() {
          return angleEncoder.getPosition();
     }

     private double getTz() {
          double[] get = LimelightHelpers.getTargetPose_RobotSpace("");
          return get.length >= 2 ? get[2] : 0.0;
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

     @Override
     public void periodic() {
          SmartDashboard.putNumber("LL Tz", getTz());
          SmartDashboard.putNumber("Angle Encoder Position", getAnglePosition());
          SmartDashboard.putNumber("Angle Velocity", getAngleSpeed());
     }
}
