package org.biobuzz.sim;

import static org.firstinspires.ftc.teamcode.field.HiveCell.A;
import static org.firstinspires.ftc.teamcode.field.HiveCell.B;
import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.field.HiveCell;
import org.junit.Test;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * HiveTipModel's exact odds against closed forms worked out here a different way (a binomial tail over every ball
 * aimed at a cell, not volley by volley) and against the research numbers (sim research digest, biobuzz-hive). The
 * route's volleys are [A, B, B, B] with 4 balls each: the preloads at A, then three pickups' worth at B.
 */
public class HiveTipModelTest {

    private static final List<HiveCell> ROUTE = Arrays.asList(A, B, B, B);
    private static final double EXACT = 1e-12;
    /** Half a unit in the 4th place: the research numbers are rounded to 4 places. */
    private static final double FOUR_PLACES = 5e-5;
    /** Half a unit in the 2nd place, for the research table's 2-place numbers. */
    private static final double TWO_PLACES = 5e-3;

    /** P(at least k of n balls go in), each independently with probability p: the closed form. */
    private static double atLeast(int k, int n, double p) {
        double sum = 0;
        for (int i = k; i <= n; i++) {
            sum += choose(n, i) * Math.pow(p, i) * Math.pow(1 - p, n - i);
        }
        return sum;
    }

    private static double choose(int n, int k) {
        double c = 1;
        for (int i = 1; i <= k; i++) {
            c = c * (n - k + i) / i;
        }
        return c;
    }

    @Test
    public void aTipsOnVolley1WhenAtLeast3Of4PreloadsGoIn() {
        List<HiveTipModel.VolleyOdds> odds = HiveTipModel.odds(ROUTE, 0.9);
        // 4 x 0.9^3 x 0.1 + 0.9^4.
        assertEquals(0.9477, odds.get(0).pTipped(A), EXACT);
        assertEquals(atLeast(3, 4, 0.9), odds.get(0).pTipped(A), EXACT);
        assertEquals(0.0, odds.get(0).pTipped(B), 0.0);
        // A gets only the preload volley, so its odds stay put after it.
        for (HiveTipModel.VolleyOdds later : odds) {
            assertEquals("volley " + later.number, 0.9477, later.pTipped(A), EXACT);
        }
    }

    @Test
    public void givenATippedOnVolley1BTipsByVolley3OnlyWhenAll8BallsGoIn() {
        List<HiveTipModel.VolleyOdds> odds = HiveTipModel.odds(ROUTE, 0.9);
        // B is down until A tips, and A can only tip on volley 1, so P(B by 3 | A) = P(B by 3) / P(A on 1).
        double given = odds.get(2).pTipped(B) / odds.get(0).pTipped(A);
        assertEquals(Math.pow(0.9, 8), given, EXACT);
        assertEquals(0.4305, given, FOUR_PLACES);
        // Volley 2's 4 balls cannot fill an empty cell that tips at 8.
        assertEquals(0.0, odds.get(1).pTipped(B), 0.0);
    }

    @Test
    public void givenATippedOnVolley1BTipsByVolley4WhenAtLeast8Of12BallsGoIn() {
        List<HiveTipModel.VolleyOdds> odds = HiveTipModel.odds(ROUTE, 0.9);
        double given = odds.get(3).pTipped(B) / odds.get(0).pTipped(A);
        assertEquals(atLeast(8, 12, 0.9), given, EXACT);
        assertEquals(0.9957, given, FOUR_PLACES);
    }

    @Test
    public void bothCellsTipByVolley4WithATimesBGivenA() {
        HiveTipModel.VolleyOdds last = HiveTipModel.odds(ROUTE, 0.9).get(3);
        assertEquals(0.9477 * atLeast(8, 12, 0.9), last.pBothTipped(), EXACT);
        assertEquals(0.9477 * 0.9957, last.pBothTipped(), FOUR_PLACES);
        // One tip for A, one more for B.
        assertEquals(0.9477 + 0.9477 * atLeast(8, 12, 0.9), last.expectedTips, EXACT);
    }

    @Test
    public void perfectHitsFlagVolley4AsADownCellVolley() {
        List<HiveTipModel.VolleyOdds> odds = HiveTipModel.odds(ROUTE, 1.0);
        // A tips on its 3rd preload (the 4th is lost), B fills to 8 over volleys 2 and 3 and tips, and A comes back
        // up: volley 4 flies at the down B.
        double[] tips = {1, 1, 2, 2};
        double[] bTipped = {0, 0, 1, 1};
        for (int i = 0; i < 4; i++) {
            HiveTipModel.VolleyOdds volley = odds.get(i);
            assertEquals(i + 1, volley.number);
            assertEquals(ROUTE.get(i), volley.aimed);
            assertEquals(4, volley.balls);
            assertEquals("volley " + volley.number, 1.0, volley.pTipped(A), 0.0);
            assertEquals("volley " + volley.number, bTipped[i], volley.pTipped(B), 0.0);
            assertEquals("volley " + volley.number, tips[i], volley.expectedTips, 0.0);
        }
        for (int i = 0; i < 3; i++) {
            assertEquals("volley " + (i + 1), 0.0, odds.get(i).pDownCell, 0.0);
            assertFalse("volley " + (i + 1), odds.get(i).flagged());
        }
        assertEquals(1.0, odds.get(3).pDownCell, 0.0);
        assertTrue(odds.get(3).flagged());
    }

    @Test
    public void aMissedFirstTipLeavesEveryBVolleyAtADownCell() {
        List<HiveTipModel.VolleyOdds> odds = HiveTipModel.odds(ROUTE, 0.9);
        double aMissed = 1 - 0.9477;
        assertEquals(0.0, odds.get(0).pDownCell, 0.0);
        assertEquals(aMissed, odds.get(1).pDownCell, EXACT);
        assertEquals(aMissed, odds.get(2).pDownCell, EXACT);
        // Volley 4 is also at a down cell when B already tipped on volley 3.
        assertEquals(aMissed + 0.9477 * Math.pow(0.9, 8), odds.get(3).pDownCell, EXACT);
        assertTrue(odds.get(1).flagged());
        assertTrue(odds.get(3).flagged());
    }

    @Test
    public void matchesTheResearchOddsAt90And80And70PercentHits() {
        double[] hitRates = {0.9, 0.8, 0.7};
        // Research table (2 places; B by volley 4 at 0.9 to 3): A tips on volley 1; B by volley 3 and by volley 4,
        // both given A tipped; both tip.
        double[] aTips = {0.95, 0.82, 0.65};
        double[] bBy3GivenA = {0.43, 0.17, 0.06};
        double[] bBy4GivenA = {0.996, 0.93, 0.72};
        double[] both = {0.94, 0.76, 0.47};
        for (int i = 0; i < hitRates.length; i++) {
            double p = hitRates[i];
            List<HiveTipModel.VolleyOdds> odds = HiveTipModel.odds(ROUTE, p);
            double a = odds.get(0).pTipped(A);
            double by3 = odds.get(2).pTipped(B) / a;
            double by4 = odds.get(3).pTipped(B) / a;
            String at = "p = " + p;
            assertEquals(at, atLeast(3, 4, p), a, EXACT);
            assertEquals(at, Math.pow(p, 8), by3, EXACT);
            assertEquals(at, atLeast(8, 12, p), by4, EXACT);
            assertEquals(at, atLeast(3, 4, p) * atLeast(8, 12, p), odds.get(3).pBothTipped(), EXACT);
            assertEquals(at, aTips[i], a, TWO_PLACES);
            assertEquals(at, bBy3GivenA[i], by3, TWO_PLACES);
            assertEquals(at, bBy4GivenA[i], by4, i == 0 ? 5e-4 : TWO_PLACES);
            assertEquals(at, both[i], odds.get(3).pBothTipped(), TWO_PLACES);
        }
    }

    @Test
    public void givenATippedThreeBallsPerPickupLeaveBNeeding8Of9() {
        int[] balls = HiveTipModel.ballsPerVolley(4, 3);
        for (double p : new double[] {0.9, 0.8, 0.7}) {
            List<HiveTipModel.VolleyOdds> odds = HiveTipModel.odds(ROUTE, balls, p);
            double given = odds.get(3).pTipped(B) / odds.get(0).pTipped(A);
            assertEquals("p = " + p, atLeast(3, 4, p), odds.get(0).pTipped(A), EXACT);
            assertEquals("p = " + p, atLeast(8, 9, p), given, EXACT);
        }
    }

    @Test
    public void theRestOfATippingVolleyIsLost() {
        // A tips on the 3rd preload; the 4th would make B's 4 + 3 into 8 if it carried over.
        List<HiveTipModel.VolleyOdds> seven = HiveTipModel.odds(Arrays.asList(A, B, B), new int[] {4, 4, 3}, 1.0);
        assertEquals(0.0, seven.get(2).pTipped(B), 0.0);
        assertEquals(1.0, seven.get(2).expectedTips, 0.0);
        // B keeps its 7 between volleys: one more ball tips it.
        List<HiveTipModel.VolleyOdds> topped = HiveTipModel.odds(ROUTE, new int[] {4, 4, 3, 1}, 1.0);
        assertEquals(1.0, topped.get(3).pTipped(B), 0.0);
        assertEquals(2.0, topped.get(3).expectedTips, 0.0);
    }

    @Test
    public void aCellThatComesBackUpIsEmptyAndTipsAt8() {
        List<HiveCell> aimed = Arrays.asList(A, B, B, A, A);
        List<HiveTipModel.VolleyOdds> odds = HiveTipModel.odds(aimed, 1.0);
        // A is back up after B tips on volley 3, without its nectar or pollen: 4 balls leave it 4 short.
        double[] tips = {1, 1, 2, 2, 3};
        for (int i = 0; i < tips.length; i++) {
            assertEquals("volley " + (i + 1), tips[i], odds.get(i).expectedTips, 0.0);
            assertEquals("volley " + (i + 1), 0.0, odds.get(i).pDownCell, 0.0);
        }
        assertEquals(0.0, odds.get(3).pAtLeastTips(3), 0.0);
        assertEquals(1.0, odds.get(4).pAtLeastTips(3), 0.0);
    }

    @Test
    public void aVolleyAtADownCellScoresNothing() {
        // Volley 1 at the down B is lost: B comes up empty after A tips and needs volleys 3 and 4.
        List<HiveTipModel.VolleyOdds> odds = HiveTipModel.odds(Arrays.asList(B, A, B, B), 1.0);
        double[] down = {1, 0, 0, 0};
        double[] tips = {0, 1, 1, 2};
        for (int i = 0; i < 4; i++) {
            assertEquals("volley " + (i + 1), down[i], odds.get(i).pDownCell, 0.0);
            assertEquals("volley " + (i + 1), tips[i], odds.get(i).expectedTips, 0.0);
        }
        assertTrue(odds.get(0).flagged());
    }

    @Test
    public void noHitsNoTipsAndEveryBVolleyIsAtADownCell() {
        List<HiveTipModel.VolleyOdds> odds = HiveTipModel.odds(ROUTE, 0.0);
        for (HiveTipModel.VolleyOdds volley : odds) {
            assertEquals(0.0, volley.expectedTips, 0.0);
            assertEquals(0.0, volley.pTipped(A), 0.0);
            assertEquals(0.0, volley.pTipped(B), 0.0);
            assertEquals(volley.aimed == B ? 1.0 : 0.0, volley.pDownCell, 0.0);
        }
    }

    @Test
    public void expectedTipsAddUpTheTipOddsAndTheOutcomesAddUpToOne() {
        List<HiveTipModel.VolleyOdds> odds = HiveTipModel.odds(Arrays.asList(A, B, B, A, A, B), 0.8);
        for (HiveTipModel.VolleyOdds volley : odds) {
            double sum = 0;
            for (int k = 1; k <= odds.size(); k++) {
                sum += volley.pAtLeastTips(k);
            }
            assertEquals("volley " + volley.number, sum, volley.expectedTips, EXACT);
            assertEquals("volley " + volley.number, 1.0, volley.pAtLeastTips(0), EXACT);
            assertEquals("volley " + volley.number, 0.0, volley.pAtLeastTips(volley.number + 1), 0.0);
        }
    }

    @Test
    public void ballsPerVolleyArePreloadsThenPickups() {
        assertArrayEquals(new int[] {4, 4, 4, 4}, HiveTipModel.ballsPerVolley(4, HiveTipModel.DEFAULT_PICKUP));
        assertArrayEquals(new int[] {4, 2, 2}, HiveTipModel.ballsPerVolley(3, 2));
        assertArrayEquals(new int[] {4}, HiveTipModel.ballsPerVolley(1, 0));
        assertArrayEquals(new int[0], HiveTipModel.ballsPerVolley(0, 4));
        assertEquals(4, HiveTipModel.PRELOADS);
        assertTrue(HiveTipModel.odds(Collections.<HiveCell>emptyList(), 0.9).isEmpty());
    }

    @Test
    public void refusesImpossibleInputs() {
        assertThrows(IllegalArgumentException.class, () -> HiveTipModel.odds(ROUTE, -0.1));
        assertThrows(IllegalArgumentException.class, () -> HiveTipModel.odds(ROUTE, 1.1));
        assertThrows(IllegalArgumentException.class, () -> HiveTipModel.odds(ROUTE, Double.NaN));
        // G407: no robot holds more than 4.
        assertThrows(IllegalArgumentException.class, () -> HiveTipModel.odds(ROUTE, new int[] {4, 5, 4, 4}, 0.9));
        assertThrows(IllegalArgumentException.class, () -> HiveTipModel.odds(ROUTE, new int[] {4, -1, 4, 4}, 0.9));
        assertThrows(IllegalArgumentException.class, () -> HiveTipModel.odds(ROUTE, new int[] {4, 4, 4}, 0.9));
        assertThrows(IllegalArgumentException.class,
                () -> HiveTipModel.odds(Arrays.asList(A, null), new int[] {4, 4}, 0.9));
        assertThrows(IllegalArgumentException.class, () -> HiveTipModel.ballsPerVolley(4, 5));
        assertThrows(IllegalArgumentException.class, () -> HiveTipModel.ballsPerVolley(4, -1));
        assertThrows(IllegalArgumentException.class, () -> HiveTipModel.ballsPerVolley(-1, 4));
        HiveTipModel.VolleyOdds first = HiveTipModel.odds(ROUTE, 0.9).get(0);
        assertThrows(IllegalArgumentException.class, () -> first.pAtLeastTips(-1));
    }
}
