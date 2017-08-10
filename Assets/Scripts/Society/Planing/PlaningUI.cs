using UnityEngine.UI;

class PlaningUI : ModuleUI {

    public static string WAITING = "Waiting";
    public static string DEFAULT_TARGET = "(0.0, 0.0)";
    public Text statusText = null;
    public Text targetText = null;
    public Button ModuleButton = null;

    private Button startButton;
    private Button returnButton;
    private Button offroadButton;

    protected override void Awake() {
        base.Awake();

        SafeInstantiateText(statusText, uiTransform, WAITING);
        SafeInstantiateText(targetText, uiTransform, DEFAULT_TARGET);

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
        Planing.singleton.Start();
    }

    public void OnClickButtonReturn() {
        Planing.singleton.ReturnToStart();
    }

    public void OnClickButtonOffroad() {
        Planing.singleton.Offroad();
    }

    protected override void Update() {
        TargetCommand? currentTarget = Planing.singleton.GetCurrentTarget();
        if (currentTarget == null) {
            statusText.text = WAITING;
            targetText.text = DEFAULT_TARGET;
        } else {
            statusText.text = currentTarget.ToString();
            targetText.text = Planing.singleton.GetCurrentTargetPosition().ToString();
        }
    }
}
