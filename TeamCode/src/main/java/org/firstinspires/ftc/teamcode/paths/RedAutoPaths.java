package org.firstinspires.ftc.teamcode.paths;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

// Visualizer export, mirrored on x and y (180 deg turn about center). Never hand-edit, re-export.
public class RedAutoPaths {

    private final PoseFactory poseFactory = PoseFactory.degrees().mirrorX(72).mirrorY(72);

    private final Pose start = poseFactory.of(79.1642, 133.0811, 0);
    private final Pose path1Start = poseFactory.of(79.1642, 133.0811, 0);
    private final Pose path1 = poseFactory.of(134.5, 134.4568, 0);
    private final Pose point2 = poseFactory.of(94.9585, 19.33356404577335, -134.6572);
    private final Pose point2Control1 = poseFactory.of(118.206, 46.4269, 0);
    private final Pose point2Segment1Start = poseFactory.of(94.9585, 19.33356404577335, 0);
    private final Pose point2Segment1End = poseFactory.of(94.9585, 19.33356404577335, 235);
    private final Pose point3 = poseFactory.of(94.4204, 12.7932, -93.2517);
    private final Pose point4 = poseFactory.of(94.8887, 23.5465, -92.749);
    private final Pose point5 = poseFactory.of(118.9252, 93.9917, -91.738);
    private final Pose point5Control1 = poseFactory.of(117.6429, 48.0847, 0);
    private final Pose point6 = poseFactory.of(128, 94.1197, 0.6519);
    private final Pose point7 = poseFactory.of(118.5731, 93.9834, 0.6731);
    private final Pose point8 = poseFactory.of(94.7675, 22.8091, 60.0839);
    private final Pose point8Control1 = poseFactory.of(124.677, 74.6391, 0);
    private final Pose point9 = poseFactory.of(133.3059, 66.7496, 90);
    private final Pose point9Control1 = poseFactory.of(136.1478, 104.8682, 0);
    private final Pose point9Segment2Heading = poseFactory.of(133.3059, 66.7496, 90);
    private final Pose point10 = poseFactory.of(133.2975, 40.2161, 90);

    public Path garden() {
        return line(path1Start, path1).linear(path1Start, path1);
    }

    public Path gardenShoot() {
        return curve(path1, point2Control1, point2).heading(Interpolator.piecewise().until(0.1854, Interpolator.linear(point2Segment1Start, point2Segment1End)).until(1, Interpolator.tangent));
    }

    public Path flower1() {
        return line(point2, point3).tangent();
    }

    public Path flower1Shoot() {
        return line(point3, point4).reverseTangent();
    }

    public Path flower2Prep() {
        return curve(point4, point5Control1, point5).reverseTangent();
    }

    public Path flower2() {
        return line(point5, point6).tangent();
    }

    public Path pushPrep() {
        return line(point6, point7).reverseTangent();
    }

    public Path flower2Shoot() {
        return curve(point7, point8Control1, point8).reverseTangent();
    }

    public Path parkPrep() {
        return curve(point8, point9Control1, point9).constant(point9Segment2Heading);
    }

    public Path park() {
        return line(point9, point10).constant(point10);
    }
}
