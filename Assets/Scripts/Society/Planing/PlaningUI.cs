using UnityEngine;
using UnityEngine.UI;

class PlaningUI : ModuleUI {

    public Text statusText = null;
    public Text targetText = null;
    public Button ModuleButton = null;

    private Button startButton;
    private Button returnButton;
    private Button offroadButton;

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

        offroadButton = SafeInstantiate<Button>(ModuleButton, uiTransform);
        offroadButton.GetComponentInChildren<Text>().text = "Offroad";
        offroadButton.onClick.AddListener(OnClickButtonOffroad);
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

    public void OnClickButtonOffroad() {
        Planing.singleton.Offroad();
    }

    protected override void Update() {
        TargetCommand currentTarget = Planing.singleton.GetCurrentTarget();
        statusText.text = currentTarget.ToString();
        targetText.text = Planing.singleton.GetCurrentTargetPosition().ToString();
    }
}
