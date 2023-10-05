package frc.lib.util;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Preferences;

public class PreferencesHelper {
    private static ArrayList<String> loadedPreferences = new ArrayList<>();

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
        if (!loadedPreferences.contains(key)) {
            Preferences.initDouble(key, backup);
            loadedPreferences.add(key);
        }
            return Preferences.getDouble(key, backup);
    }
}
