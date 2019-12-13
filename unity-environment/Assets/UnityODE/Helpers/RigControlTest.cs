using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using AaltoGames;

public class RigControlTest : MonoBehaviour {
    public bool fixCharacterInAir = true;
    public float[] targetPose;
    MecanimODERig rig;
	// Use this for initialization
	void Start () {
        if (ODEManager.instance == null)
            Debug.LogException(new System.Exception("ODE Manager missing, cannot create ODE objects!"));

        rig = GetComponent<MecanimODERig>();
        if (rig == null)
            Debug.LogException(new System.Exception("This script needs a MecanimODERig component on the same game object"));
        rig.initialize();
        targetPose=new float[rig.numControlDOFs()];
        if (fixCharacterInAir)
        {
            int jointId = UnityOde.odeJointCreateFixed();
            UnityOde.odeJointAttach(jointId, 0, rig.bones[0].rb.BodyId);
            UnityOde.odeJointSetFixed(jointId);
        }
    }
    void FixedUpdate()
    {
        UnityOde.setCurrentOdeContext(ODEManager.instance.masterContext);
        for (int i = 0; i < targetPose.Length; i++)
        {
            targetPose[i] = Mathf.Clamp(targetPose[i], rig.angleMinLimits()[i], rig.angleMaxLimits()[i]);
        }
        rig.driveToPose(targetPose, Time.fixedDeltaTime);
        
        Debug.Log("Control effort: " + rig.getControlEffort());
    }
}
