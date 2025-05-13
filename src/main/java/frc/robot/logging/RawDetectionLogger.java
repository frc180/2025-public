package frc.robot.logging;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import frc.robot.util.LimelightHelpers.RawDetection;

@CustomLoggerFor(RawDetection.class)
public class RawDetectionLogger extends ClassSpecificLogger<RawDetection> {

    public RawDetectionLogger() {
        super(RawDetection.class);
    }

    @Override
    public void update(EpilogueBackend backend, RawDetection detection) {
        backend.log("classId", detection.classId);
        backend.log("txnc", detection.txnc);
        backend.log("tync", detection.tync);
        backend.log("ta", detection.ta);
    }
}