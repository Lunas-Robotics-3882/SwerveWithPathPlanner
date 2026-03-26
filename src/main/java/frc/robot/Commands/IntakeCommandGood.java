package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class IntakeCommandGood extends Command{

    private Timer timer = new Timer();

     private IndexerSubsystem indexer = new IndexerSubsystem();
     private IntakeSubsystem intake = new IntakeSubsystem();

    private boolean firstcheck = true;

    public IntakeCommandGood(IndexerSubsystem indexer, IntakeSubsystem intake) {
        this.indexer = indexer;
        this.intake = intake;
        addRequirements(indexer, intake);

      }

      @Override
  public void initialize() {
    timer.restart();
    timer.start();
    intake.intake();
    indexer.indexerintake();
  }

  @Override
  public void execute()
  {
    if (timer.get() > .3)
    {
      indexer.disable();
      
    }
  }

      @Override
      public boolean isFinished() {
        return false;
      }

     @Override
     public void end(boolean interrupted) {
      indexer.disable();
      intake.disable();
    }
}