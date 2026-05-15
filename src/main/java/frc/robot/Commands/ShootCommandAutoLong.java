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


public class ShootCommandAutoLong extends Command{

    private Timer timer = new Timer();

     private IndexerSubsystem indexer = new IndexerSubsystem();
     private ShooterSubsystem shooter = new ShooterSubsystem();
     private FeederSubsystem feeder = new FeederSubsystem();
     private IntakeSubsystem intake = new IntakeSubsystem();
    
    private boolean firstcheck = true;

    public ShootCommandAutoLong(ShooterSubsystem shooter,IndexerSubsystem indexer, FeederSubsystem feeder, IntakeSubsystem intake) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.feeder = feeder;
        this.intake = intake;
        addRequirements(shooter,feeder,indexer, intake);

      }

      @Override
  public void initialize() {
    timer.restart();
    timer.start();
    shooter.shooter();
  }

  @Override
  public void execute()
  {
    if (timer.get() > 0.33)
    {
      indexer.index();
      feeder.feeder();
    }

    if (timer.get() > 0.7)
    {
      intake.intake();
    }
  }

      @Override
      public boolean isFinished() {

        if (timer.get() > 5.5)
        {
          return true;
        }

        return false;
      }

     @Override
     public void end(boolean interrupted) {
      shooter.disable();
      indexer.disable();
      feeder.disable();
    }
}