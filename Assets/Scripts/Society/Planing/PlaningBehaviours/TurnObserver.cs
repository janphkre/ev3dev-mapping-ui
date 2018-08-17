using UnityEngine;
using System.Collections;
using ev3devMapping;
using ev3dev.Society;

class TurnObserver {


    private CarDrive steering;
    private PositionHistory positionHistory;

    private volatile AbstractSteering requestedCommand = new IdleSteering();

    public TurnObserver(CarDrive steering, PositionHistory positionHistory) {
        this.steering = steering;
        this.positionHistory = positionHistory;
    }

    public IEnumerator turnRoutine() {
        while (true) {
            yield return new WaitWhile(() => requestedCommand is IdleSteering);
            AbstractSteering currentCommand = requestedCommand;
            currentCommand.Execute();
        }
    }

    public void turn() {
        
    }

    public void turnStop() {
        
    }

    public void stop() {

    }
}
