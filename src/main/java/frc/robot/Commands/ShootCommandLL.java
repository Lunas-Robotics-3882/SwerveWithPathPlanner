package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.DoubleSupplier;

public class ShootCommandLL extends Command {

    private final Timer timer = new Timer();
     private IndexerSubsystem indexer = new IndexerSubsystem();
     private ShooterSubsystem shooter = new ShooterSubsystem();
     private FeederSubsystem feeder = new FeederSubsystem();
    private final DoubleSupplier distanceSupplier;

    /**
     * @param distanceSupplier A function that returns the current distance (usually from drivetrain pose)
     */
    public ShootCommandLL(ShooterSubsystem shooter, IndexerSubsystem indexer, FeederSubsystem feeder, DoubleSupplier distanceSupplier) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.feeder = feeder;
        this.distanceSupplier = distanceSupplier;
        
        addRequirements(shooter, feeder, indexer);
    }

    @Override
    public void initialize() {
        timer.restart();
        // Set initial speed based on current distance
        double currentDistance = distanceSupplier.getAsDouble();
        shooter.shooter(currentDistance); 
    }

    @Override
    public void execute() {
        // Continuously update shooter speed in case the robot is moving while shooting
        double currentDistance = distanceSupplier.getAsDouble();
        shooter.shooter(currentDistance);

        // Wait for flywheel spin-up before feeding
        if (timer.get() > 0.2) {
            indexer.index();
            feeder.feeder();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.disable();
        indexer.disable();
        feeder.disable();
        timer.stop();
    }
}