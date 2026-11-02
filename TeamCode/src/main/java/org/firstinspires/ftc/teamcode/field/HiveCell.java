package org.firstinspires.ftc.teamcode.field;

/** The two cells of our hive. A starts up, B starts down. A tip swaps which one is up. */
public enum HiveCell {
    A,
    B;

    public HiveCell other() {
        return this == A ? B : A;
    }
}
