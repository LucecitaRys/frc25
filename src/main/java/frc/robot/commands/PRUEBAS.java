// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ControlBoard;
import frc.robot.subsystems.Brazo;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PRUEBAS extends Command {
  /** Creates a new PRUEBAS. */
  private final ElevatorSub mElevador = ElevatorSub.getInstance(); 
  private final Shooter mShooter = Shooter.getInstance();
  private final Brazo mBrazo = Brazo.getInstance();
  public PRUEBAS() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mBrazo);
addRequirements(mElevador);
addRequirements(mShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
     mBrazo.getposbrazo(ControlBoard.getLeftY_ope());
     mElevador.GETPOSESELEVATOR(ControlBoard.getRightY_ope());
  mShooter.GETPOSESMU(ControlBoard.ButtonMuL());
  mShooter.GETPOSESMU(ControlBoard.ButtonMuR());
  SmartDashboard.putNumber("POSE ELEVATOR", mElevador.getPosEle());
  SmartDashboard.putNumber("POSE BRAZO", mBrazo.getPoseB());
  SmartDashboard.putNumber("POSE MUÑECA", mShooter.getposm());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
