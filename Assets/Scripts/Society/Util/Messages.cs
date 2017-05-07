using UnityEngine;
using UnityEngine.Networking;

public enum MessageType {
    LocalClientMap,
    GlobalClientMap,
    Quit
};

public class PointMessage : MessageBase {

    public Vector3[] cloud;
    public Vector3 robotPos;

    public PointMessage() { }

    public PointMessage(Vector3[] data, Vector3 robotPos) {
        cloud = new Vector3[data.Length];
        for (int i = 0; i < data.Length; i++) {
            cloud[i] = data[i];
        }
    }

    public PointMessage(Vector3[] data, int len, Vector3 robotPos) {
        cloud = new Vector3[len];
        for(int i = 0; i < len; i++) {
            cloud[i] = data[i];
        }
    }

    public PointMessage(Vector3[] data, int i_from, int len, bool[] is_invalid, Vector3 robotPos) {
        int realLength = 0;
        for(int i = i_from; i < i_from + len; i++) {
            if (!is_invalid[i]) realLength++;
        }
        cloud = new Vector3[realLength];
        int j = 0;
        for (int i = i_from; i < i_from+len; i++) {
            if (!is_invalid[i]) {
                cloud[j] = data[i];
                j++;
            }
        }
    }
}