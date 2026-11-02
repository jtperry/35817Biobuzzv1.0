package org.firstinspires.ftc.teamcode.paths;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

// Visualizer export. Never hand-edit, re-export.
public class BlueAuto1TipPaths {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(78.5983, 133.1597, 90);
    private final Pose path1 = poseFactory.of(133, 78.0676, -90);
    private final Pose path1Segment1Heading = poseFactory.of(133, 78.0676, 270);
    private final Pose point2 = poseFactory.of(132.2702, 42.8006, -91.1854);

    public Path path1() {
        return line(start, path1).heading(Interpolator.piecewise().until(1, Interpolator.constant(path1Segment1Heading)));
    }

    public Path path2() {
        return line(path1, point2).tangent();
    }
}
