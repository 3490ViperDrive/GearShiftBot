package frc.lib.util;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Preferences;

public class PreferencesHelper {
    private static ArrayList<String> loadedDoublePreferences = new ArrayList<>();
    private static ArrayList<String> loadedBooleanPreferences = new ArrayList<>();

    private PreferencesHelper() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Basically Preferences.getDouble() but it will automatically initialize the items in the table if they don't exist
     * @param key the key
     * @param backup the initial value if this item is uninitialized
     * @return the value for the given key in the Preferences table if it exists, or the backup if it hasn't been initialized yet
     */
    public static double grabDouble(String key, double backup) {
        if (!loadedDoublePreferences.contains(key)) {
            Preferences.initDouble(key, backup);
            loadedDoublePreferences.add(key);
        }
            return Preferences.getDouble(key, backup);
    }

    /**
     * Basically Preferences.getBoolean() but it will automatically initialize the items in the table if they don't exist
     * @param key the key
     * @param backup the initial value if this item is uninitialized
     * @return the value for the given key in the Preferences table if it exists, or the backup if it hasn't been initialized yet
     */
    public static boolean grabBoolean(String key, boolean backup) {
        if (!loadedBooleanPreferences.contains(key)) {
            Preferences.initBoolean(key, backup);
            loadedBooleanPreferences.add(key);
        }
            return Preferences.getBoolean(key, backup);
    }
}
