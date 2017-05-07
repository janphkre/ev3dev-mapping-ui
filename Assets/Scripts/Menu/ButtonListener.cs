using UnityEngine;
using UnityEngine.Networking;

class ButtonListener: MonoBehaviour {

    private NetworkDiscovery discovery;

    public void Start() {
        discovery = GameObject.Find("Society").GetComponent<NetworkDiscovery>();
        discovery.Initialize();
    }

    public void onClickButtonStartClient() {
        discovery.StartAsClient();

    }

    public void onClickButtonStartServer() {
        NetworkManager.singleton.StartServer();
        discovery.StartAsServer();
    }
}