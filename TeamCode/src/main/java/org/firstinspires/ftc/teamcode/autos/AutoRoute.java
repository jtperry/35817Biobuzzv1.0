package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.paths.BlueAutoPaths;
import org.firstinspires.ftc.teamcode.paths.RedAutoPaths;

/** Paths MainAuto2Tip uses. */
public interface AutoRoute {

    // One method per path, same name as visualizer.
    Path intakeflower();

    Path gardenhiveBshoot();

    Path hiveBshootflower1();

    Path flower1hiveBshootcheck();

    Path hiveBshootcheckflower2prep();

    Path flower2prepflower2();

    Path flower2hiveAshoot();

    Path hiveAshootparkprep();

    Path parkpreppark();

    // Adapters. Inherit export methods. Renamed path = compile error here.
    final class Red extends RedAutoPaths implements AutoRoute {
    }

    final class Blue extends BlueAutoPaths implements AutoRoute {
    }
}
