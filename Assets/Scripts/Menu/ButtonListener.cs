using UnityEngine;
using UnityEngine.Networking;

class ButtonListener: MonoBehaviour {

    private NetworkDiscovery discovery;

    public void Start() {
        discovery = GameObject.Find("Society").GetComponent<SocietyDiscovery>();
        discovery.Initialize();
    }

    public void onClickButtonStartClient() {
        if(MainMenu.TransferInput()) discovery.StartAsClient();

    }

    public void onClickButtonStartServer() {
        NetworkManager.singleton.StartServer();
        discovery.StartAsServer();
    }

}