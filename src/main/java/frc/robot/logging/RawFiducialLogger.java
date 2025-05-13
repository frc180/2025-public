package frc.robot.logging;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import frc.robot.util.LimelightHelpers.RawFiducial;

@CustomLoggerFor(RawFiducial.class)
public class RawFiducialLogger extends ClassSpecificLogger<RawFiducial> {

    public RawFiducialLogger() {
        super(RawFiducial.class);
    }

    @Override
    public void update(EpilogueBackend backend, RawFiducial fiducial) {
        backend.log("id", fiducial.id);
        backend.log("txnc", fiducial.txnc);
        backend.log("tync", fiducial.tync);
        backend.log("ta", fiducial.ta);
        backend.log("distToCamera", fiducial.distToCamera);
        backend.log("distToRobot", fiducial.distToRobot);
        backend.log("ambiguity", fiducial.ambiguity);
    }
}