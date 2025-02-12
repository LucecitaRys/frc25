// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import com.fasterxml.jackson.annotation.JsonFormat.Shape;

import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ControlBoard;

import frc.robot.subsystems.Brazo;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.ElevatorSub.ElePoses;
import frc.robot.subsystems.Shooter.intake_states;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Brazo.brazoposes;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ModeCorall extends Command {
  private final ElevatorSub mElevador = ElevatorSub.getInstance(); 
  private final Shooter mShooter = Shooter.getInstance();
  private final Brazo mBrazo = Brazo.getInstance();
  
  private boolean mFlag;
 
  public ModeCorall() {
addRequirements(mBrazo, mElevador,mElevador);
  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(ControlBoard.ButtonCORAL()){
      if (ControlBoard.buttonA()) {
        mBrazo.brStates = brazoposes.collectCoral;
        mElevador.ElPos = ElePoses.collect;
        mShooter.inStates = intake_states.collectCoral;
        
        Shuffleboard.addEventMarker("CoralCollect", EventImportance.kHigh);
      }
      if (ControlBoard.buttonB()) {
        mBrazo.brStates = brazoposes.nivel1;
        mElevador.ElPos = ElePoses.nivel1;
        mShooter.inStates = intake_states.throwCoral;
        Shuffleboard.addEventMarker("Coral1", EventImportance.kHigh);
      }
      if (ControlBoard.buttonx()) {
        mBrazo.brStates = brazoposes.nivel2;
        mElevador.ElPos = ElePoses.nivel2;
        mShooter.inStates = intake_states.throwCoral;
        Shuffleboard.addEventMarker("Coral2", EventImportance.kHigh);
      }
      if (ControlBoard.buttony()) {
        mBrazo.brStates = brazoposes.nivel3;
        mElevador.ElPos = ElePoses.nivel3;
        mShooter.inStates = intake_states.throwCoral;
        Shuffleboard.addEventMarker("Coral3", EventImportance.kHigh);
      }
      if (ControlBoard.button5()) {
        mBrazo.brStates = brazoposes.nivel4;
        mElevador.ElPos = ElePoses.nivel4;
        mShooter.inStates = intake_states.throwCoral;
        Shuffleboard.addEventMarker("Coral4", EventImportance.kHigh);
      }
    }
    if(mShooter.inStates == mShooter.inStates.collectCoral)
    {   
      if(mBrazo.posb== mBrazo.getPoseB()&& mShooter.posm == mShooter.getposm() && mElevador.posel== mElevador.getPosEle()){

        mShooter.setConstantVel(0.5);
      
        if(mShooter.CurrentL()>= 0.32) {
        mShooter.setConstantVel(0);
        mFlag= true;
      }
      }

      }

      if(mShooter.inStates == mShooter.inStates.throwCoral)
    {   
      if(mBrazo.posb== mBrazo.getPoseB()&& mShooter.posm == mShooter.getposm() && mElevador.posel== mElevador.getPosEle()){

        mShooter.setConstantVel(-0.5);
       
        if(mShooter.CurrentL()< 0.32) {
          mFlag = true;
       
      }
      }
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mFlag;
  }
}

