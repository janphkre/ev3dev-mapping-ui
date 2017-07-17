using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Networking;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class ServerMap : NetworkBehaviour {

    //Every connection has its own coordinate system. Therefor the different clouds have to be merged through a filter into a complete map.
    private Dictionary<int, ServerClientItem> clientMaps;
    private int cloudCount;
    private PairingLocalization pairingLocalization;
        
    private class ServerClientItem {
        internal GlobalClientMap clientMap;

        internal ServerClientItem(GlobalClientMap clientMap) {
            this.clientMap = clientMap;
        }

        internal int GetFeatureCount() {
            return clientMap.globalStateVector.Count - clientMap.globalStateCollection.Count;
        }
    }

    public override void OnStartServer() {
        base.OnStartServer();
        //NetworkServer.RegisterHandler((short)MessageType.LocalClientMap, OnLocalClientMap);
        NetworkServer.RegisterHandler((short)MessageType.GlobalClientMap, OnGlobalClientMap);
        NetworkServer.RegisterHandler((short)MessageType.Quit, OnQuitMessage);
        clientMaps = new Dictionary<int, ServerClientItem>();
        cloudCount = 0;
        pairingLocalization = new PairingLocalization();
    }

    /*void OnLocalClientMap(NetworkMessage netMsg) {
        Debug.Log("Recieved Local Client Map Message from " + netMsg.conn.connectionId);
        LocalClientMap msg = netMsg.ReadMessage<LocalClientMap>();
        ServerClientItem value = null;
        if (!clientMaps.TryGetValue(netMsg.conn.connectionId, out value)) {
            cloudCount++;
            value = new ();
            clientMaps.Add(netMsg.conn.connectionId, value);
        }
        StartCoroutine(value.ConsumeLocalMap(msg));
    }*/

    void OnGlobalClientMap(NetworkMessage netMsg) {
        Debug.Log("Recieved Map Message from " + netMsg.conn.connectionId);
        GlobalClientMap msg = netMsg.ReadMessage<GlobalClientMap>();
        ServerClientItem value = null;
        if (!clientMaps.TryGetValue(netMsg.conn.connectionId, out value)) {
            cloudCount++;
            value = new ServerClientItem(msg);
            clientMaps.Add(netMsg.conn.connectionId, value);
        } else {
            value.clientMap = msg;
        }

    }

    void OnWifiMessage(NetworkMessage netMsg) {
        Debug.Log("Recieved Wifi Message from " + netMsg.conn.connectionId);
        /*PointMessage msg = netMsg.ReadMessage<PointMessage>();
        wifiCloud.AddPoints(netMsg.conn.connectionId, msg.GetPointCloud());*/
    }

    void OnQuitMessage(NetworkMessage netMsg) {
        Debug.Log("Recieved Quit Message from " + netMsg.conn.connectionId);
        if (clientMaps.Remove(netMsg.conn.connectionId)) cloudCount--;//TODO: does the map from that client really have to be deleted? -> it is some sort of knowledge that can still be used by the other robots.
    }

    internal int GetMinimumFeatureCount() {
        int result = int.MaxValue;
        foreach (KeyValuePair<int, ServerClientItem> cloudPair in clientMaps) {
            var currentCount = cloudPair.Value.GetFeatureCount();
            result = result > currentCount ? currentCount : result;
        }
        return result;
    }

    public void MergeSubClouds() {
        foreach(KeyValuePair<int, ServerClientItem> pair in clientMaps) {
            Vector3 match = pairingLocalization.MatchServer(pair.Value.clientMap.globalStateVector, pair.Value.clientMap.globalStateVector);
            Vector3 localMatchOffset = new Vector3(match.x - localMap.points.end.x, match.y - localMap.points.end.y, match.z - localMap.points.end.z);
        }
    }
}