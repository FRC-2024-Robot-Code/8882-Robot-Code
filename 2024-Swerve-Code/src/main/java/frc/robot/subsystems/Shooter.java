package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  CANSparkMax BlackShooter, GreenShooter, conveyor;

  public Shooter() {
    BlackShooter = new CANSparkMax(13, MotorType.kBrushless);
    GreenShooter = new CANSparkMax(14, MotorType.kBrushless);
    conveyor = new CANSparkMax(15, MotorType.kBrushless);

    GreenShooter.setInverted(true);
    BlackShooter.setInverted(true);

    conveyor.setInverted(true);
    GreenShooter.follow(BlackShooter);
  }

  public void setSpeed(double speed) {
    BlackShooter.set(speed);
  }

  public void stopMotor() {
    BlackShooter.stopMotor();
  }

  public void setSpeedConveyor(double speed) {
    conveyor.set(speed);
  }

  public void stopMotorConveyor() {
    conveyor.stopMotor();
  }

  public void stopAll() {
    BlackShooter.stopMotor();
    conveyor.stopMotor();
  }

}
