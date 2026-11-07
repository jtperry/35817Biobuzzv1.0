package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.math.Pose;

import java.lang.reflect.Field;

/** Read "start" pose out of visualizer export. */
final class StartPose {

    private StartPose() {
    }

    /** Export "start" pose. Throw if export has none. */
    static Pose of(Object paths) {
        for (Class<?> type = paths.getClass(); type != null; type = type.getSuperclass()) {
            try {
                Field field = type.getDeclaredField("start");
                field.setAccessible(true);
                return (Pose) field.get(paths);
            } catch (NoSuchFieldException e) {
            } catch (IllegalAccessException e) {
                throw new IllegalStateException("Cannot read start pose from export", e);
            }
        }
        throw new IllegalStateException("Export has no 'start' pose");
    }
}
