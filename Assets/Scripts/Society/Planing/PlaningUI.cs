using UnityEngine;
using UnityEngine.UI;

namespace ev3devMapping.Society {

class PlaningUI : ModuleUI {

    public Text statusText = null;
    public Text targetText = null;
    public Button ModuleButton = null;

    private Button startButton;
    private Button returnButton;

    protected override void Awake() {

        uiTransform = Instantiate<Transform>(UiTransform);
        moduleName = SafeInstantiateText(ModuleName, uiTransform, "module");
        control = transform.parent.GetComponentInChildren<Control>();
        module = GetComponent<RobotModule>();

        SafeInstantiateText(statusText, uiTransform, "Waiting");
        SafeInstantiateText(targetText, uiTransform, "(0.0, 0.0)");

        startButton = SafeInstantiate<Button>(ModuleButton, uiTransform);
        startButton.GetComponentInChildren<Text>().text = "Start";
        startButton.onClick.AddListener(OnClickButtonStart);

        returnButton = SafeInstantiate<Button>(ModuleButton, uiTransform);
        returnButton.GetComponentInChildren<Text>().text = "Return to Start";
        returnButton.onClick.AddListener(OnClickButtonReturn);
    }

    protected override void Start() {
        moduleName.text = "Planing";
    }

    public void OnClickButtonStart() {
        Planing.singleton.StartPlaning();
    }

    public void OnClickButtonReturn() {
        Planing.singleton.ReturnToStart();
    }

    protected override void Update() {
        TargetCommand currentTarget = Planing.singleton.GetCurrentTarget();
        statusText.text = currentTarget.ToString();
        targetText.text = Planing.singleton.GetCurrentTargetPosition().ToString();
    }
}
}