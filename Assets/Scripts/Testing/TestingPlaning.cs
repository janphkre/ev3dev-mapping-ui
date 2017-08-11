using UnityEngine;

//This script is attached to the "Robot" GameObject.

class TestingPlaning : MonoBehaviour {

    internal Planing planing;
    internal PlaningInputData sampleInput;

    public void Start() {
        planing = gameObject.GetComponentInChildren<Planing>();
        sampleInput = new PlaningInputData();
        sampleInput.Read();
        //Perform tests here:

    }
}
