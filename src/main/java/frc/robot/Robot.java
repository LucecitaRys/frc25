package frc.robot;

import java.util.Optional;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Drive.DriveControlMode;
import frc.robot.auto.IAuto;

import frc.robot.subsystems.Brazo;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ElevatorSub;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Brazo.brazoposes;
import frc.robot.subsystems.Drive.DriveControlState;
import frc.robot.subsystems.ElevatorSub.ElePoses;
import frc.robot.subsystems.Shooter.intake_states;


public class Robot extends TimedRobot {
  private Telemetry mTelemetry;
  private Drive mDrive;
  private Shooter mShooter;
  private ElevatorSub mElevatorSub;
  private Brazo mBrazo;
  // private Intake mIntake;
  // private Shooter mShooter;
  // private Climber mClimber;
  private Optional<IAuto> mAutoMode = Optional.empty();
  private Command mAutonomousCommand;


  @Override
  public void robotInit() {
    mTelemetry = new Telemetry();
    mDrive = Drive.getInstance();
    mShooter = Shooter.getInstance();
    mBrazo = Brazo.getInstance();
    mElevatorSub = ElevatorSub.getInstance();
    // mIntake = Intake.getInstance()
    // mShooter = Shooter.getInstance();
    // mClimber = Climber.getInstance();
    // CameraServer.startAutomaticCapture("camera", 0);
    
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    mDrive.setDriveControlState(DriveControlState.None);
    mBrazo.brStates = brazoposes.none;
   mElevatorSub.ElPos = ElePoses.none;
   mShooter.inStates = intake_states.none;
    // mShooter.setShooterControlState(ShooterControlState.None);
    // mClimber.setControlState(ClimberControlState.None);
  }

  @Override
  public void disabledPeriodic() {
    mTelemetry.updateAutoModeCreator();
    mAutoMode = mTelemetry.getAutoModeSelected();
  }

  @Override
  public void autonomousInit() {
    if (mAutoMode.isPresent()) {
      mDrive.setKinematicsLimits(Constants.Drive.uncappedLimits);
      mDrive.resetOdometry(mAutoMode.get().getStartingPose());
      mAutonomousCommand = mAutoMode.get().getAutoCommand();
      mAutonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (mAutonomousCommand != null) {
      mAutonomousCommand.cancel();
    }
    mDrive.setKinematicsLimits(Constants.Drive.oneMPSLimits);
    mDrive.setDriveControlState(DriveControlState.TeleopControl);
    // mIntake.setControlState(IntakeControlState.VariableVelocity);
    // mShooter.setShooterControlState(ShooterControlState.VariableVelocity);
    // mClimber.setControlState(ClimberControlState.PercentOutput);
  }

  @Override
  public void teleopPeriodic() {
    if (ControlBoard.driver.getXButtonPressed()) {
      mDrive.setYawAngle(0); 
    }
    if (ControlBoard.driver.getPOV() != -1) { 
      mDrive.setHeadingControl(Rotation2d.fromDegrees(ControlBoard.driver.getPOV())); 
    } 
    
    if(ControlBoard.ButtonCORAL()){
      if (ControlBoard.buttonA()) {
        mBrazo.brStates = brazoposes.collectCoral;
        mElevatorSub.ElPos = ElePoses.collect;
        mShooter.inStates = intake_states.collectCoral;
      }
      if (ControlBoard.buttonB()) {
        mBrazo.brStates = brazoposes.nivel1;
        mElevatorSub.ElPos = ElePoses.nivel1;
        mShooter.inStates = intake_states.throwCoral;
      }
      if (ControlBoard.buttonx()) {
        mBrazo.brStates = brazoposes.nivel2;
        mElevatorSub.ElPos = ElePoses.nivel2;
        mShooter.inStates = intake_states.throwCoral;
      }
      if (ControlBoard.buttony()) {
        mBrazo.brStates = brazoposes.nivel3;
        mElevatorSub.ElPos = ElePoses.nivel3;
        mShooter.inStates = intake_states.throwCoral;
      }
      if (ControlBoard.button5()) {
        mBrazo.brStates = brazoposes.nivel4;
        mElevatorSub.ElPos = ElePoses.nivel4;
        mShooter.inStates = intake_states.throwCoral;
      }
    }



    if(ControlBoard.ButtonALGAE()){
      if (ControlBoard.buttonA()) {
        mBrazo.brStates = brazoposes.collectAlgae;
        mElevatorSub.ElPos = ElePoses.collect;
        mShooter.inStates = intake_states.collectAlgae;
      }
      if (ControlBoard.buttonB()) {
        mBrazo.brStates = brazoposes.nivel1;
        mElevatorSub.ElPos = ElePoses.nivel1;
        mShooter.inStates = intake_states.collectAlgae;
      }
      if (ControlBoard.buttonx()) {
        mBrazo.brStates = brazoposes.nivel1;
        mElevatorSub.ElPos = ElePoses.nivel2;
        mShooter.inStates = intake_states.collectAlgae;
      }
      if (ControlBoard.buttony()) {
        mBrazo.brStates = brazoposes.nivel3;
        mElevatorSub.ElPos = ElePoses.nivel3;
        mShooter.inStates = intake_states.throwAlagae;
      }
      
    }

    
  }
  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
  }
