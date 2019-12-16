using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HoldInfo : MonoBehaviour {

    public enum HoldType {Sphere, Cube};

    public int holdId = -1;
    public HoldType targetHoldType = HoldType.Sphere;
    private HoldType currentHoldType = HoldType.Sphere;

    List<Transform> childTransforms = new List<Transform>();

    public void SetHoldType(HoldType InHoldType)
    {
        if (currentHoldType == InHoldType)
            return;
        switch (InHoldType)
        {
            case HoldType.Cube:
                childTransforms[0].gameObject.SetActive(false);
                childTransforms[1].gameObject.SetActive(true);
                break;
            case HoldType.Sphere:
                childTransforms[0].gameObject.SetActive(true);
                childTransforms[1].gameObject.SetActive(false);
                break;
            default:
                break;
        }

        currentHoldType = InHoldType;
        return;
    }
    // Use this for initialization
    void Start () {
        
        for (int i = 0; i < this.transform.childCount; i++)
        {
            childTransforms.Add(this.transform.GetChild(i));
        }
    }
	
	// Update is called once per frame
	void Update () {
        SetHoldType(targetHoldType);

    }
}
