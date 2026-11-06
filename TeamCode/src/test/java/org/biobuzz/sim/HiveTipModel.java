package org.biobuzz.sim;

import org.firstinspires.ftc.teamcode.field.HiveCell;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Exact odds that our hive tips, volley by volley, for a list of volleys (the cell each aims at, the balls it
 * launches). No randomness: every outcome is weighed by its probability, so a hit rate is a parameter and the sim
 * never has to run again to sweep it.
 *
 * <p>The hive, on a calibrated field (Event Field Setup Guide V1.0 section 12.3; manual sections 10.3.1 and 10.5.1).
 * The cell that starts up (A) holds the 3 staged nectar and tips at its 3rd pollen; every cell that comes up later is
 * empty and tips at its 8th. On a tip the cell empties, the other (empty) cell comes up, and the rest of that volley is
 * lost (a modelling choice, as in Zenith: the balls behind the tipping one spill with the cell). A volley aimed at a
 * down cell scores nothing and is flagged: launching into a down cell's outside faces is a G417 note (decision 41 keeps
 * the route's volleys and accepts the risk). Each ball goes in with probability p, independently of every other, so
 * the odds are binomial; misses that come together (one bad aim misses a whole volley) make them an upper bound.
 *
 * <p>The state is the number of tips so far and the pollen in the up cell. Tips alternate A, B, A, ..., so the count
 * says which cell is up, whether it still has the staged nectar (only before the first tip), and which cells have
 * tipped.
 */
public final class HiveTipModel {

    /** The cell that starts up tips at its 3rd pollen, on its 3 staged nectar (Field Setup Guide section 12.3). */
    public static final int FIRST_TIP_POLLEN = 3;
    /** Every later up cell is empty and tips at its 8th pollen (section 12.3). */
    public static final int LATER_TIP_POLLEN = 8;
    /** Every robot starts with exactly 4 preloaded pollen (manual section 10.3.4): the first volley launches them. */
    public static final int PRELOADS = 4;
    /** G407: a robot controls at most 4 scoring elements, so a volley launches at most 4. */
    public static final int MAX_HELD = 4;
    /** Pollen each pickup collects unless told otherwise: a full load. A guess until the robot runs the route. */
    public static final int DEFAULT_PICKUP = 4;

    private HiveTipModel() {
    }

    /** Balls each volley launches: the preloads, then pickup for every later volley (each one follows a pickup). */
    public static int[] ballsPerVolley(int volleys, int pickup) {
        if (volleys < 0) {
            throw new IllegalArgumentException(volleys + " volleys");
        }
        checkBalls("a pickup", pickup);
        int[] balls = new int[volleys];
        for (int i = 0; i < volleys; i++) {
            balls[i] = i == 0 ? PRELOADS : pickup;
        }
        return balls;
    }

    /** The odds after each volley at hit rate p: the preloads, then DEFAULT_PICKUP balls per later volley. */
    public static List<VolleyOdds> odds(List<HiveCell> aimed, double p) {
        return odds(aimed, ballsPerVolley(aimed.size(), DEFAULT_PICKUP), p);
    }

    /** The odds after each volley at hit rate p: volley i aims at aimed.get(i) and launches balls[i]. */
    public static List<VolleyOdds> odds(List<HiveCell> aimed, int[] balls, double p) {
        if (!(p >= 0 && p <= 1)) {
            throw new IllegalArgumentException("hit rate " + p + " is not in [0, 1]");
        }
        if (aimed.size() != balls.length) {
            throw new IllegalArgumentException(aimed.size() + " aimed cells for " + balls.length + " volleys");
        }
        int volleys = balls.length;
        // weight[tips][pollen]: P(that many tips so far and that much pollen in the up cell). A volley tips the hive at
        // most once (the rest of it is lost), and the up cell always holds less than it tips at.
        double[][] weight = new double[volleys + 1][LATER_TIP_POLLEN];
        weight[0][0] = 1;
        List<VolleyOdds> odds = new ArrayList<>();
        for (int v = 0; v < volleys; v++) {
            HiveCell cell = aimed.get(v);
            if (cell == null) {
                throw new IllegalArgumentException("volley " + (v + 1) + " aims at no cell");
            }
            int n = balls[v];
            checkBalls("volley " + (v + 1), n);
            double[][] next = new double[volleys + 1][LATER_TIP_POLLEN];
            double down = 0;
            for (int tips = 0; tips <= v; tips++) {
                for (int pollen = 0; pollen < LATER_TIP_POLLEN; pollen++) {
                    double w = weight[tips][pollen];
                    if (w == 0) {
                        continue;
                    }
                    if (cell != upCell(tips)) {
                        // At a down cell: nothing goes in, nothing changes.
                        down += w;
                        next[tips][pollen] += w;
                        continue;
                    }
                    int missing = tipsAt(tips) - pollen;
                    for (int hits = 0; hits <= n; hits++) {
                        double share = w * binomial(n, hits, p);
                        if (hits >= missing) {
                            // Tips: the cell empties, the other (empty) one comes up, the rest of the volley is lost.
                            next[tips + 1][0] += share;
                        } else {
                            next[tips][pollen + hits] += share;
                        }
                    }
                }
            }
            weight = next;
            double[] exactly = new double[volleys + 1];
            for (int tips = 0; tips <= volleys; tips++) {
                for (int pollen = 0; pollen < LATER_TIP_POLLEN; pollen++) {
                    exactly[tips] += weight[tips][pollen];
                }
            }
            odds.add(new VolleyOdds(v + 1, cell, n, down, exactly));
        }
        return Collections.unmodifiableList(odds);
    }

    /** The cell that is up after this many tips: A starts up and each tip swaps. */
    static HiveCell upCell(int tips) {
        return tips % 2 == 0 ? HiveCell.A : HiveCell.B;
    }

    /** Pollen the up cell tips at after this many tips: only the first one has the staged nectar. */
    static int tipsAt(int tips) {
        return tips == 0 ? FIRST_TIP_POLLEN : LATER_TIP_POLLEN;
    }

    /** P(exactly k of n independent balls go in) at hit rate p. */
    private static double binomial(int n, int k, double p) {
        double ways = 1;
        for (int i = 1; i <= k; i++) {
            ways = ways * (n - k + i) / i;
        }
        return ways * Math.pow(p, k) * Math.pow(1 - p, n - k);
    }

    private static void checkBalls(String what, int balls) {
        if (balls < 0 || balls > MAX_HELD) {
            throw new IllegalArgumentException(what + " has " + balls + " balls; G407 allows 0 to " + MAX_HELD);
        }
    }

    /** The odds at the end of one volley. */
    public static final class VolleyOdds {
        /** 1 for the first volley. */
        public final int number;
        public final HiveCell aimed;
        public final int balls;
        /** P(the aimed cell is down when this volley flies): it scores nothing (G417 note). */
        public final double pDownCell;
        /** Expected number of tips by the end of this volley. */
        public final double expectedTips;
        /** P(exactly k tips by the end of this volley), k = 0 to the number of volleys. */
        private final double[] exactly;

        VolleyOdds(int number, HiveCell aimed, int balls, double pDownCell, double[] exactly) {
            this.number = number;
            this.aimed = aimed;
            this.balls = balls;
            this.pDownCell = pDownCell;
            this.exactly = exactly;
            double expected = 0;
            for (int k = 1; k < exactly.length; k++) {
                expected += k * exactly[k];
            }
            this.expectedTips = expected;
        }

        /** P(at least k tips by the end of this volley). */
        public double pAtLeastTips(int k) {
            if (k < 0) {
                throw new IllegalArgumentException(k + " tips");
            }
            double sum = 0;
            for (int i = k; i < exactly.length; i++) {
                sum += exactly[i];
            }
            return sum;
        }

        /** P(this cell has tipped by the end of this volley): A's first tip is the first tip, B's the second. */
        public double pTipped(HiveCell cell) {
            return pAtLeastTips(cell == HiveCell.A ? 1 : 2);
        }

        /** P(both cells have tipped by the end of this volley): the same as B, which stays down until A tips. */
        public double pBothTipped() {
            return pAtLeastTips(2);
        }

        /** True when this volley can fly at a down cell (G417 note), however unlikely. */
        public boolean flagged() {
            return pDownCell > 0;
        }
    }
}
