// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Brazo;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Brazo.brazoposes;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Autonomos extends Command {
   private final ElevatorSub mElevador = ElevatorSub.getInstance(); 
  private final Shooter mShooter = Shooter.getInstance();
  private final Brazo mBrazo = Brazo.getInstance();
 
 private Brazo.brazoposes mypose = null;
 private Shooter.intake_states MyposeSho= null;
 private ElevatorSub.ElePoses MyposeEl= null;
 private boolean mFlag;

  
  public Autonomos(Brazo.brazoposes pose, Shooter.intake_states poseSho, ElevatorSub.ElePoses poseEle) {
    mypose = pose;
    MyposeSho = poseSho;
    MyposeEl = poseEle;
    addRequirements(mBrazo);
addRequirements(mElevador);
addRequirements(mShooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mBrazo.brStates = mypose;
    mShooter.inStates = MyposeSho;
    mElevador.ElPos = MyposeEl;


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mShooter.inStates == mShooter.inStates.collectAlgae) {
      if (mBrazo.posb == mBrazo.getPoseB() && mShooter.posm == mShooter.getposm()
          && mElevador.posel == mElevador.getPosEle()) {

        mShooter.setConstantVel(0.5);

        if (mShooter.CurrentL() >= 0.32) {
          mShooter.setConstantVel(0);
          mFlag = true;
        }
      }

    }

    if (mShooter.inStates == mShooter.inStates.throwAlagae) {
      if (mBrazo.posb == mBrazo.getPoseB() && mShooter.posm == mShooter.getposm()
          && mElevador.posel == mElevador.getPosEle()) {

        mShooter.setConstantVel(-0.5);

        if (mShooter.CurrentL() < 0.32) {
          mShooter.setConstantVel(-0.5);
          mFlag = true;

        }
      }
    }
    if (mShooter.inStates == mShooter.inStates.collectCoral) {
      if (mBrazo.posb == mBrazo.getPoseB() && mShooter.posm == mShooter.getposm()
          && mElevador.posel == mElevador.getPosEle()) {

        mShooter.setConstantVel(0.5);

        if (mShooter.CurrentL() >= 0.32) {
          mShooter.setConstantVel(0);
          mFlag = true;
        }
      }

    }

    if (mShooter.inStates == mShooter.inStates.throwCoral) {
      if (mBrazo.posb == mBrazo.getPoseB() && mShooter.posm == mShooter.getposm()
          && mElevador.posel == mElevador.getPosEle()) {

        mShooter.setConstantVel(-0.5);

        if (mShooter.CurrentL() < 0.32) {
          mFlag = true;

        }
      }
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mFlag;
  }
}
