using System;
using UnityEngine;

class CarDrive: MonoBehaviour {

    public void Awake() {
        
    }

    public void Halt() {
        throw new NotImplementedException();
    }

    public void MoveBackward() {
        throw new NotImplementedException();
    }

    public bool IsMovingBackwards() {
        throw new NotImplementedException();
    }

    public bool IsMoving() {
        throw new NotImplementedException();
    }

    public void Steer(float segment, bool backwards) {
        throw new NotImplementedException();
    }

    //Moves forward for the segment of the max steering angle circle and stops afterwards.
    public void SteerForward(float segment) {
        throw new NotImplementedException();
    }

    //Same as SteerForward but backwards
    public void SteerBackwards(float segment) {
        throw new NotImplementedException();
    }
}
