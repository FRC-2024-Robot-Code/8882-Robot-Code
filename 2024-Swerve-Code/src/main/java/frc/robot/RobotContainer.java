// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.StadiaController.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Controle;
import frc.robot.Constants.Trajetoria;
import frc.robot.commands.AngleAmp;
import frc.robot.commands.AngleCmd;
import frc.robot.commands.AngleSpeaker;
import frc.robot.commands.Collect;
import frc.robot.commands.LimelightAlign;
import frc.robot.commands.Teleop;
import frc.robot.commands.Auto.CollectAuto;
import frc.robot.commands.Auto.Speaker1;
import frc.robot.commands.Auto.Angles.AngleForIntake;
import frc.robot.commands.Auto.Angles.Return0;
import frc.robot.commands.Auto.Angles.MidAuto.Angle1;
import frc.robot.commands.Auto.Angles.MidAuto.Angle2;
import frc.robot.commands.Auto.Angles.MidAuto.Angle3;
import frc.robot.commands.Auto.Angles.MidAuto.Angle5;
import frc.robot.commands.Auto.Angles.SourceAuto.sAngle1;
import frc.robot.commands.Auto.Angles.SourceAuto.sAngle2;
import frc.robot.commands.Shoots.Amp;
import frc.robot.commands.Shoots.Speaker;
import frc.robot.subsystems.Angle;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import java.io.File;

public class RobotContainer {

  static final Swerve swerve = new Swerve(
      new File(Filesystem.getDeployDirectory(), "swerve"));
  public static final Intake intake = new Intake();
  public static final Angle angle = new Angle();
  public static final Shooter shooter = new Shooter();

  public static final XboxController driverControl = new XboxController(
      Controle.xboxControle);
  public static final Joystick operatorControl = new Joystick(
      Controle.controle2);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    swerve.setDefaultCommand(
        new Teleop(
            swerve,
            () -> -MathUtil.applyDeadband(driverControl.getLeftY(), Controle.DEADBAND),
            () -> -MathUtil.applyDeadband(driverControl.getLeftX(), Controle.DEADBAND),
            () -> -MathUtil.applyDeadband(driverControl.getRightX(), Controle.DEADBAND),
            driverControl));

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    NamedCommands.registerCommand("cancel", Commands.run(() -> cancel()));

    NamedCommands.registerCommand("angleForIntake", new AngleForIntake(angle));
    NamedCommands.registerCommand("intake", new CollectAuto(intake, shooter));
    NamedCommands.registerCommand("stopIntake", Commands.run(() -> {
      intake.stopIntake();
      shooter.stopConveyor();
    }, intake, shooter));
    NamedCommands.registerCommand("stopAngle", Commands.run(() -> angle.stopAngle(), angle));
    NamedCommands.registerCommand("return0", new Return0(angle));

    NamedCommands.registerCommand("angle1", new Angle1(angle));
    NamedCommands.registerCommand("angle2", new Angle2(angle));
    NamedCommands.registerCommand("angle3", new Angle3(angle));
    NamedCommands.registerCommand("angle4", new Angle5(angle));
    NamedCommands.registerCommand("angle5", new Angle5(angle));

    NamedCommands.registerCommand("mShoot1", new Speaker1(shooter, 0.5, 0.7, 1, 0.5));
    NamedCommands.registerCommand("mShoot2", new Speaker1(shooter, 0.40, 0.7, 1, 1));
    NamedCommands.registerCommand("mShoot3", new Speaker1(shooter, 0.40, 0.7, 1.5, 0.5));

    NamedCommands.registerCommand("sAngle1", new sAngle1(angle));
    NamedCommands.registerCommand("sAngle2", new sAngle2(angle));

    NamedCommands.registerCommand("sShoot1", new Speaker1(shooter, 0.40, 0.7, 2.5, 1));
    NamedCommands.registerCommand("sShoot2", new Speaker1(shooter, 0.4, 0.7, 4, 1));

    configureBindings();
  }

  private void configureBindings() {

    new JoystickButton(driverControl, Button.kA.value)
        .onTrue(new InstantCommand(swerve::zeroGyro));

    new JoystickButton(driverControl, XboxController.Button.kX.value).onTrue(new LimelightAlign(swerve, driverControl));

    new JoystickButton(operatorControl,
        XboxController.Button.kA.value).onTrue(new AngleCmd(angle, 0.5));
    new JoystickButton(operatorControl, XboxController.Button.kA.value).whileTrue(new Collect(intake, shooter))
        .whileFalse(Commands.runOnce(() -> {
          shooter.stopConveyor();
          intake.stopIntake();
        }, shooter, intake));

    new JoystickButton(operatorControl, XboxController.Button.kX.value).onTrue(new AngleSpeaker(angle));

    new JoystickButton(operatorControl, XboxController.Button.kY.value).onTrue(new AngleCmd(angle, 0.85));

    new JoystickButton(operatorControl, XboxController.Button.kStart.value).whileTrue(Commands.startEnd(() -> {
      intake.invertIntake();
      shooter.setConveyorSpeed(-0.2);
      shooter.setShooterSpeed(-0.2);
    }, () -> {
      intake.stopIntake();
      shooter.stopConveyor();
      shooter.stopShooter();
    }, intake, shooter));

    new JoystickButton(operatorControl,
        XboxController.Button.kB.value).onTrue(new AngleCmd(angle, 0.32));

    // new JoystickButton(operatorControl,
    // XboxController.Button.kB.value).onTrue(new AngleCmd(angle, 0.21));

    new JoystickButton(operatorControl, XboxController.Button.kBack.value).onTrue(Commands.run(() -> {
      shooter.stopConveyor();
      shooter.stopShooter();
      intake.stopIntake();
    }, shooter, intake));

    new Trigger(this::getRight).onTrue(new Speaker(shooter, 0.45, 0.6, 2, 1));
    new Trigger(this::getLeft).onTrue(new Amp(shooter));
    new Trigger(this::getLeft).onTrue(new AngleAmp(angle, 0.10));

  }

  private boolean getRight() {
    if (operatorControl.getRawAxis(Controle.rightTrigger) != 0) {
      return true;
    }
    return false;
  }

  private boolean getLeft() {
    if (operatorControl.getRawAxis(Controle.leftTrigger) != 0) {
      return true;
    }
    return false;

  }

  private boolean getTOP() {
    if (operatorControl.getPOV() == 0) {
      return true;
    }
    return false;
  }

  private void cancel() {
    this.cancel();
  }

  public Command getAutonomousCommand() {
    // return autoChooser.getSelected();
    return swerve.getAutonomousCommand(Trajetoria.AUTO_SOURCE_1, true, true);
  }

  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }

  public void setHeadingCorrection(boolean headingCorrection) {
    swerve.swerveDrive.setHeadingCorrection(headingCorrection);
  }

}
