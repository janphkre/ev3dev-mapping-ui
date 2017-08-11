using UnityEngine.Networking;

namespace ev3devMapping {

class SocietyDiscovery : NetworkDiscovery {

    public override void OnReceivedBroadcast(string fromAddress, string data) {
        NetworkManager.singleton.networkAddress = fromAddress;
        NetworkManager.singleton.StartClient();
        StopBroadcast();
    }
}
}
