package org.firstinspires.ftc.teamcode.paths;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

// Visualizer export, mirrored on x and y (180 deg turn about center). Never hand-edit, re-export.
public class RedAutoPaths {

    private final PoseFactory poseFactory = PoseFactory.degrees().mirrorX(72).mirrorY(72);

    private final Pose start = poseFactory.of(110.3, 133.21, 90);
    private final Pose intakeflower = poseFactory.of(133.445, 133.21, 90);
    private final Pose intakeflowerControl1 = poseFactory.of(131.5447, 100.3648, 0);
    private final Pose gardenhiveBshoot = poseFactory.of(94.7215, 22.3935, -133.9836);
    private final Pose gardenhiveBshootControl1 = poseFactory.of(116.7011, 45.1671, 0);
    private final Pose gardenhiveBshootSegment1Start = poseFactory.of(94.7215, 22.3935, 90);
    private final Pose gardenhiveBshootSegment1End = poseFactory.of(94.7215, 22.3935, 160);
    private final Pose hiveBshootflower1 = poseFactory.of(94.72, 11.48, 270);
    private final Pose flower1hiveBshootcheck = poseFactory.of(94.72, 22.39, 270);
    private final Pose hiveBshootcheckflower2prep = poseFactory.of(118.8208, 94.5322, 92.4668);
    private final Pose hiveBshootcheckflower2prepControl1 = poseFactory.of(121.4084, 34.4681, 0);
    private final Pose flower2prepflower2 = poseFactory.of(132.52, 94.53, 0);
    private final Pose flower2hiveAshoot = poseFactory.of(94.4753, 15.3491, 11.6182);
    private final Pose flower2hiveAshootControl1 = poseFactory.of(101.1296, 102.1329, 0);
    private final Pose flower2hiveAshootControl2 = poseFactory.of(134.3687, 23.5513, 0);
    private final Pose hiveAshootparkprep = poseFactory.of(125.8028, 78.6836, -151.6714);
    private final Pose hiveAshootparkprepControl1 = poseFactory.of(122.3286, 76.8106, 0);
    private final Pose parkpreppark = poseFactory.of(127.8285, 48.3734, 93.8234);

    public Path intakeflower() {
        return Paths.curve(start, intakeflowerControl1, intakeflower).constant(intakeflower);
    }

    public Path gardenhiveBshoot() {
        return Paths.curve(intakeflower, gardenhiveBshootControl1, gardenhiveBshoot).heading(Interpolator.piecewise().until(0.25, Interpolator.linear(gardenhiveBshootSegment1Start, gardenhiveBshootSegment1End)).until(1, Interpolator.tangent));
    }

    public Path hiveBshootflower1() {
        return Paths.line(gardenhiveBshoot, hiveBshootflower1).constant(hiveBshootflower1);
    }

    public Path flower1hiveBshootcheck() {
        return Paths.line(hiveBshootflower1, flower1hiveBshootcheck).constant(flower1hiveBshootcheck);
    }

    public Path hiveBshootcheckflower2prep() {
        return Paths.curve(flower1hiveBshootcheck, hiveBshootcheckflower2prepControl1, hiveBshootcheckflower2prep).tangent();
    }

    public Path flower2prepflower2() {
        return Paths.line(hiveBshootcheckflower2prep, flower2prepflower2).constant(flower2prepflower2);
    }

    public Path flower2hiveAshoot() {
        return Paths.curve(flower2prepflower2, flower2hiveAshootControl1, flower2hiveAshootControl2, flower2hiveAshoot).reverseTangent();
    }

    public Path hiveAshootparkprep() {
        return Paths.curve(flower2hiveAshoot, hiveAshootparkprepControl1, hiveAshootparkprep).reverseTangent();
    }

    public Path parkpreppark() {
        return Paths.line(hiveAshootparkprep, parkpreppark).reverseTangent();
    }
}
