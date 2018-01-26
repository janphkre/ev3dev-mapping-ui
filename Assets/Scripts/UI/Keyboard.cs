using UnityEngine;
using System.Collections;

namespace ev3devMapping {

public class Keyboard : MonoBehaviour
{
	void Update ()
	{
		if (Input.GetKeyDown (KeyCode.Escape))
			SceneManager.Instance.ToggleShowUI ();
	}
}
}