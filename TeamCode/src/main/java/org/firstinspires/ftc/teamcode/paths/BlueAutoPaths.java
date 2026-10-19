package org.firstinspires.ftc.teamcode.paths;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

// Visualizer export. Never hand-edit, re-export.
public class BlueAutoPaths {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(111.0017, 134.4568, 0);
    private final Pose path1Start = poseFactory.of(111.0017, 134.4568, 0);
    private final Pose path1 = poseFactory.of(133.5, 134.4568, 0);
    private final Pose point2 = poseFactory.of(94.9585, 23.2641, -134.6572);
    private final Pose point2Control1 = poseFactory.of(118.206, 46.4269, 0);
    private final Pose point2Segment1Start = poseFactory.of(94.9585, 23.2641, 0);
    private final Pose point2Segment1End = poseFactory.of(94.9585, 23.2641, 235);
    private final Pose point3 = poseFactory.of(95.0432, 13.2807, -89.5138);
    private final Pose point4 = poseFactory.of(94.8887, 23.5465, -89.1378);
    private final Pose point5 = poseFactory.of(118.9252, 93.9917, -91.738);
    private final Pose point5Control1 = poseFactory.of(117.6429, 48.0847, 0);
    private final Pose point6 = poseFactory.of(128.6761, 94.3688, 2.2146);
    private final Pose point7 = poseFactory.of(118.5731, 93.9834, 2.1845);
    private final Pose point8 = poseFactory.of(130.3916, 38.2545, -92.0092);
    private final Pose point8Control1 = poseFactory.of(131.7769, 75.7601, 0);

    public Path path1() {
        return line(path1Start, path1).linear(path1Start, path1);
    }

    public Path path2() {
        return curve(path1, point2Control1, point2).heading(Interpolator.piecewise().until(0.1854, Interpolator.linear(point2Segment1Start, point2Segment1End)).until(1, Interpolator.tangent));
    }

    public Path path3() {
        return line(point2, point3).tangent();
    }

    public Path path4() {
        return line(point3, point4).reverseTangent();
    }

    public Path path5() {
        return curve(point4, point5Control1, point5).reverseTangent();
    }

    public Path path6() {
        return line(point5, point6).tangent();
    }

    public Path path7() {
        return line(point6, point7).reverseTangent();
    }

    public Path path8() {
        return curve(point7, point8Control1, point8).tangent();
    }
}
