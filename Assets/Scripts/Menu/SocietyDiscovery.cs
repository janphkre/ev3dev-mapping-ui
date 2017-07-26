using UnityEngine.Networking;

class SocietyDiscovery : NetworkDiscovery {

    public override void OnReceivedBroadcast(string fromAddress, string data) {
        NetworkManager.singleton.networkAddress = fromAddress;
        NetworkManager.singleton.StartClient();
        StopBroadcast();
    }
}
