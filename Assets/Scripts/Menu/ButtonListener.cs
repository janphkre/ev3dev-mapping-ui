using UnityEngine;
using UnityEngine.Networking;

namespace ev3devMapping {

class ButtonListener: MonoBehaviour {

    private NetworkDiscovery discovery;

    public void Start() {
        discovery = GameObject.Find("Society").GetComponent<SocietyDiscovery>();
        discovery.Initialize();
    }

    public void onClickButtonStartClient() {
        if (MainMenu.TransferInput()) {
            MainMenu.IsNetworkingEnabled = true;
            discovery.StartAsClient();
        }
    }

    public void onClickButtonStartServer() {
        MainMenu.IsNetworkingEnabled = true;
        NetworkManager.singleton.StartServer();
        discovery.StartAsServer();
    }

    public void onClickButtonStartSolo() {
        if (MainMenu.TransferInput()) {
            UnityEngine.SceneManagement.SceneManager.LoadSceneAsync("Base");
            
        }
    }
}
}
