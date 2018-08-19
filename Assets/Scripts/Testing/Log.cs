using System.IO;
using UnityEngine;

class Log {
    private const string LOG_FILE = "Planing.log";

    public static void log(string message) {
        Debug.Log(message);
        if (!File.Exists(LOG_FILE)) File.Create(LOG_FILE).Dispose();
        
        using (var writer = new StreamWriter(File.OpenWrite(LOG_FILE))) {
            writer.WriteLine(message);
            writer.Flush();
            writer.Close();
            writer.Dispose();
        }
    }

    public static void logWarning(string message) {
        Debug.LogWarning(message);
        if (!File.Exists(LOG_FILE)) File.Create(LOG_FILE).Dispose();
        
        using (var writer = new StreamWriter(File.OpenWrite(LOG_FILE))) {
            writer.WriteLine(message);
            writer.Flush();
            writer.Close();
            writer.Dispose();
        }
    }
}
