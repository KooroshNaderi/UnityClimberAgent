using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System.IO;

public static class ConfigurableJointExtensions
{
    /// <summary>
    /// Sets a joint's targetRotation to match a given local rotation.
    /// The joint transform's local rotation must be cached on Start and passed into this method.
    /// </summary>
    public static void SetTargetRotationLocal(this ConfigurableJoint joint, Quaternion targetLocalRotation, Quaternion startLocalRotation)
    {
        if (joint.configuredInWorldSpace)
        {
            Debug.LogError("SetTargetRotationLocal should not be used with joints that are configured in world space. For world space joints, use SetTargetRotation.", joint);
        }
        SetTargetRotationInternal(joint, targetLocalRotation, startLocalRotation, Space.Self);
    }

    /// <summary>
    /// Sets a joint's targetRotation to match a given world rotation.
    /// The joint transform's world rotation must be cached on Start and passed into this method.
    /// </summary>
    public static void SetTargetRotation(this ConfigurableJoint joint, Quaternion targetWorldRotation, Quaternion startWorldRotation)
    {
        if (!joint.configuredInWorldSpace)
        {
            Debug.LogError("SetTargetRotation must be used with joints that are configured in world space. For local space joints, use SetTargetRotationLocal.", joint);
        }
        SetTargetRotationInternal(joint, targetWorldRotation, startWorldRotation, Space.World);
    }

    static void SetTargetRotationInternal(this ConfigurableJoint joint, Quaternion targetRotation, Quaternion startRotation, Space space)
    {
        // Calculate the rotation expressed by the joint's axis and secondary axis
        var right = joint.axis;
        var forward = Vector3.Cross(joint.axis, joint.secondaryAxis).normalized;
        var up = Vector3.Cross(forward, right).normalized;
        Quaternion worldToJointSpace = Quaternion.LookRotation(forward, up);

        // Transform into world space
        Quaternion resultRotation = Quaternion.Inverse(worldToJointSpace);

        // Counter-rotate and apply the new local rotation.
        // Joint space is the inverse of world space, so we need to invert our value
        if (space == Space.World)
        {
            resultRotation *= startRotation * Quaternion.Inverse(targetRotation);
        }
        else
        {
            resultRotation *= Quaternion.Inverse(targetRotation) * startRotation;
        }

        // Transform back into joint space
        resultRotation *= worldToJointSpace;

        // Set target rotation to our newly calculated rotation
        joint.targetRotation = resultRotation;
    }
}

public class HumanoidRigControlTest : MonoBehaviour {
    public bool fixCharacterInAir = true;
    public bool savePoseToFile = false;
    public Vector3[] targetCurrentPose;
    HumanoidMecanimRig rig;
    public GameObject mTextWriterObj;

    Text mTextWriter;
    // Use this for initialization
    void Start ()
    {
        Time.fixedDeltaTime = 1 / 100.0f;
        rig = GetComponent<HumanoidMecanimRig>();
        if (rig == null)
            Debug.LogException(new System.Exception("This script needs a HumanoidMecanimRig component on the same game object"));
        rig.Initialize();

        targetCurrentPose = new Vector3[rig.numControlDOFs()];
        readPoseAngleFromFile();

        mTextWriter = mTextWriterObj.GetComponent<Text>();
    }

    void Update()
    {
        rig.freezeBody(fixCharacterInAir);

        if (savePoseToFile)
        {
            writePoseAngleToFile();
            savePoseToFile = false;
        }
    }

    void FixedUpdate()
    {
        if (targetCurrentPose.Length < rig.numControlDOFs())
            targetCurrentPose = new Vector3[rig.numControlDOFs()];

        float[] targetPose = new float[rig.numControlDOFs()];
        for (int i = 0; i < rig.numControlDOFs(); i++)
        {
            targetPose[i] = targetCurrentPose[i][0];
        }
        rig.driveToPose(targetPose);//, 1f);
        for (int i = 0; i < rig.numControlDOFs(); i++)
        {
            targetCurrentPose[i][0] = targetPose[i];
        }

        float[] readPose = new float[rig.numControlDOFs()];
        rig.poseToMotorAngles(ref readPose);
        for (int i = 0; i < rig.numControlDOFs(); i++)
        {
            targetCurrentPose[i][1] = readPose[i];
        }

        float[] readTorques = new float[rig.numControlDOFs()];
        rig.getCurrentAplliedTorques(ref readTorques);

        float[] readForces = new float[rig.numControlDOFs()];
        rig.getCurrentAplliedForces(ref readForces);

        float sum_values = 0.0f;
        for (int i = 0; i < rig.numControlDOFs(); i++)
        {
            targetCurrentPose[i][2] = readTorques[i]; //+ readForces[i];
            sum_values += targetCurrentPose[i][2];
        }

        //      for (int i = 0; i < targetPose.Length; i++)
        //      {
        //          targetPose[i] = Mathf.Clamp(targetPose[i], rig.angleMinLimits()[i], rig.angleMaxLimits()[i]);
        //      }
        //      rig.driveToPose(targetPose, Time.fixedDeltaTime);

        mTextWriter.text = "Control effort: " + sum_values;
    }

    void writePoseAngleToFile()
    {
        FileStream posefile = new FileStream("climberAnglePos.txt", FileMode.Create);
        StreamWriter writer = new StreamWriter(posefile);
        for (int i = 0; i < rig.numControlDOFs(); i++)
        {
            writer.WriteLine(targetCurrentPose[i][0].ToString());
        }
        writer.Flush();
        writer.Close();
        posefile.Close();
    }

    public void readPoseAngleFromFile()
    { 
        FileStream posefile = new FileStream("climberAnglePos.txt", FileMode.Open);
        StreamReader reader = new StreamReader(posefile);
        for (int i = 0; i < rig.numControlDOFs(); i++)
        {
            string _str = reader.ReadLine();
            try
            {
                targetCurrentPose[i][0] = float.Parse(_str);
            }
            catch (System.Exception e)
            {
                reader.Close();
                posefile.Close();
                return;
            }
        }
        reader.Close();
        posefile.Close();
        return;
    }
}
