using UnityEngine;
using UnityEngine.Networking;

class ButtonListener: MonoBehaviour {

    private NetworkDiscovery discovery;

    public void Start() {
        discovery = GameObject.Find("Society").GetComponent<SocietyDiscovery>();
        discovery.Initialize();
    }

    public void onClickButtonStartClient() {
        //MainMenu.CheckInputs
        discovery.StartAsClient();

    }

    public void onClickButtonStartServer() {
        NetworkManager.singleton.StartServer();
        discovery.StartAsServer();
    }

}