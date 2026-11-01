package org.firstinspires.ftc.teamcode.paths;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

// Same export as Blue far 1 tip, mirrored on x and y (180 deg turn about center). Never hand-edit, re-export.
public class RedAuto1TipFarPaths {

    private final PoseFactory poseFactory = PoseFactory.degrees().mirrorX(72).mirrorY(72);

    private final Pose start = poseFactory.of(78.4245, 8.1739, 90);
    private final Pose path1 = poseFactory.of(126.9239, 23.6579, 180);
    private final Pose path1Control1 = poseFactory.of(84.1134, 41.8199, 0);

    public Path path1() {
        return curve(start, path1Control1, path1).linear(start, path1);
    }
}
