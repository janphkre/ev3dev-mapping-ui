using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Networking;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class ServerMap : NetworkBehaviour {

    //every connection has its own coordinate system. Therefor the different clouds have to be merged through a filter into a complete map.
    //private Dictionary<int, GlobalClientMap> clientMaps;
    private int cloudCount;

    public override void OnStartServer() {
        base.OnStartServer();
        NetworkServer.RegisterHandler((short)MessageType.LocalClientMap, OnLocalClientMap);
        NetworkServer.RegisterHandler((short)MessageType.GlobalClientMap, OnGlobalClientMap);
        NetworkServer.RegisterHandler((short)MessageType.Quit, OnQuitMessage);
        clientMaps = new Dictionary<int, GlobalClientMap>();
        cloudCount = 0;
    }

    void OnLocalClientMap(NetworkMessage netMsg) {
        Debug.Log("Recieved Local Client Map Message from " + netMsg.conn.connectionId);
        LocalClientMap msg = netMsg.ReadMessage<LocalClientMap>();
        ClientBundle value = null;
        if (!clientMaps.TryGetValue(netMsg.conn.connectionId, out value)) {
            cloudCount++;
            value = new ();
            clientMaps.Add(netMsg.conn.connectionId, value);
        }
        StartCoroutine(value.ConsumeLocalMap(msg));
    }

    void OnGlobalClientMap(NetworkMessage netMsg) {
        Debug.Log("Recieved Map Message from " + netMsg.conn.connectionId);
        /*PointMessage msg = netMsg.ReadMessage<PointMessage>();
        mapCloud.AddPoints(netMsg.conn.connectionId, msg.GetPointCloud());*/
    }

    void OnWifiMessage(NetworkMessage netMsg) {
        Debug.Log("Recieved Wifi Message from " + netMsg.conn.connectionId);
        /*PointMessage msg = netMsg.ReadMessage<PointMessage>();
        wifiCloud.AddPoints(netMsg.conn.connectionId, msg.GetPointCloud());*/
    }

    void OnQuitMessage(NetworkMessage netMsg) {
        Debug.Log("Recieved Quit Message from " + netMsg.conn.connectionId);
        if (clientMaps.Remove(netMsg.conn.connectionId)) cloudCount--;
    }

    public int GetMinimumPoints() {
        int result = int.MaxValue;
        foreach (KeyValuePair<int, ClientBundle> cloudPair in clientMaps) result = result > cloudPair.Value.GetPointCount() ? cloudPair.Value.GetPointCount() : result;
        return result;
    }

    public void MergeSubClouds() {
        
    }
}
