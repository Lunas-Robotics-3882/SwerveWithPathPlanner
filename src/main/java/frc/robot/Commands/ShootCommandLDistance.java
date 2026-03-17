package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.DoubleSupplier; // <--- 1. MUST HAVE THIS


public class ShootCommandLDistance extends Command {

    private final Timer timer = new Timer();
    private final IndexerSubsystem indexer;
    private final ShooterSubsystem shooter;
    private final FeederSubsystem feeder;
    private final DoubleSupplier distanceSupplier; // <--- 2. MUST HAVE THIS

    // 3. CONSTRUCTOR NAME MUST MATCH CLASS NAME EXACTLY
    public ShootCommandLDistance(
        ShooterSubsystem shooter, 
        IndexerSubsystem indexer, 
        FeederSubsystem feeder, 
        DoubleSupplier distanceSupplier
    ) 
    {
        this.shooter = shooter;
        this.indexer = indexer;
        this.feeder = feeder;
        this.distanceSupplier = distanceSupplier;
        addRequirements(shooter, feeder, indexer);
    }

    @Override
    public void initialize() {
        timer.restart();
        // Use the supplier to get the current distance
        shooter.shooter(distanceSupplier.getAsDouble());
    }

    @Override
    public void execute() {
        // Update speed live in case the robot moves
        shooter.shooter(distanceSupplier.getAsDouble());

        if (timer.get() > 0.2) {
            indexer.index();
            feeder.feeder();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.disable();
        indexer.disable();
        feeder.disable();
    }
}