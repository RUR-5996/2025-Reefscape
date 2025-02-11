package frc.robot.util;

public class ScoringTracker {
    // collumn according to Figure 5-8: Reef Scoring Location Tracking (a = 0, b = 1, ...)
    // level 0 - 3
    // 0 = empty; 1 = occupied by algae; 2 = scored

    static int[] starting_algae_high = {0, 0, 1, 0}; //algea blocking levels

    static int[] starting_algae_low = {0, 1, 1, 0}; //algea blocking levels

    static int[][] blue = {starting_algae_high, starting_algae_high, starting_algae_low, starting_algae_low, starting_algae_high, starting_algae_high, starting_algae_low, starting_algae_low, starting_algae_high, starting_algae_high, starting_algae_low, starting_algae_low};
    static int[][] red = {starting_algae_high, starting_algae_high, starting_algae_low, starting_algae_low, starting_algae_high, starting_algae_high, starting_algae_low, starting_algae_low, starting_algae_high, starting_algae_high, starting_algae_low, starting_algae_low};

    static int[][][] reefs = {blue, red};



    public static int getStatusOfReefLocation(int collumn, int level, String side) { // side is blue or red, returned  = wrong parameters
        int side_int;
        if (side == "blue")
            side_int = 0;
        else if (side == "red")
            side_int = 1;
        else
            return -1;

        return reefs[side_int][collumn][level];
    }

    public static int getStatusOfReefLocation(int collumn, int level, int side_int) { // side is 0 for blue or 1 for red, returned 3 = wrong parameters
        return reefs[side_int][collumn][level];
    }

    public static void removeAlgae(int collumn, String side) { //collumn: algae takes up two, any one of them can be inputed refer to Figure 5-8: Reef Scoring Location Tracking for specific algae locations and column naming
        int side_int;
        int collumn_modulo;
        int second_collumn;
        if (side == "blue")
            side_int = 0;
        else if (side == "red")
            side_int = 1;
        else
            return;
        
        collumn_modulo = collumn % 2;

        if (collumn_modulo == 0) {
            second_collumn = collumn + 1;
        }
        else
            second_collumn = collumn - 1;
        
        if (reefs[side_int][collumn][1] == 1) {
            reefs[side_int][collumn][1] = 0;
        }
        reefs[side_int][collumn][2] = 0;

        if (reefs[side_int][second_collumn][1] == 1) {
            reefs[side_int][second_collumn][1] = 0;
        }
        reefs[side_int][second_collumn][2] = 0;

        return;
    }

    public static void removeAlgae(int collumn, int side_int) {
        int collumn_modulo;
        int second_collumn;
        collumn_modulo = collumn % 2;

        if (collumn_modulo == 0) {
            second_collumn = collumn + 1;
        }
        else
            second_collumn = collumn - 1;
        
        if (reefs[side_int][collumn][1] == 1) {
            reefs[side_int][collumn][1] = 0;
        }
        reefs[side_int][collumn][2] = 0;

        if (reefs[side_int][second_collumn][1] == 1) {
            reefs[side_int][second_collumn][1] = 0;
        }
        reefs[side_int][second_collumn][2] = 0;

        return;
    }

    public static int checkReefLocation(int collumn, int level, String side) {
        int side_int;
        if (side == "blue")
            side_int = 0;
        else if (side == "red")
            side_int = 1;
        else
            return -1;

        return reefs[side_int][collumn][level];
    }

    public static int checkReefLocation(int collumn, int level, int side_int) {
        return reefs[side_int][collumn][level];
    }
}

