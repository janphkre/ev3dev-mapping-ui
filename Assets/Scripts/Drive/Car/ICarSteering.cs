using UnityEngine;
using System.Collections;

public interface ICarSteering {
    void Halt();
    void Steer(float segment, bool backwards);
    void SteerForward(float segment);
    void SteerBackward(float segment);
    void DriveAhead(bool backwards);
}
