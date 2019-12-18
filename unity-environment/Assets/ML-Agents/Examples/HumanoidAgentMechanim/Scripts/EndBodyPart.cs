using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class EndBodyPart : MonoBehaviour
{
    public int current_hold_id = -1;
    //public int target_hold_id = -1;
    [HideInInspector] public Transform _bodyPartInfo;
    private Rigidbody _locker_body;
    private Joint _joint;

    public bool IsConnected { get; private set; } = false;

    public Vector3 Pos
    {
        get { return _bodyPartInfo.transform.localPosition; }
    }

    // Start is called before the first frame update
    void Start()
    {
        _bodyPartInfo = this.transform;

        _locker_body = _bodyPartInfo.GetComponent<Rigidbody>();

        _joint = _bodyPartInfo.GetComponent<ConfigurableJoint>();

        //holdContact = _bodyPartInfo.GetComponent<HoldContact>();

        //groundContact = _bodyPartInfo.GetComponent<GroundContact>();
        //groundContact.agent = _p;

        //wallContact = _bodyPartInfo.GetComponent<WallContact>();

        disconnectBodyPart();
    }

    // Update is called once per frame
    void Update()
    {

    }

    public Vector3 getCurrentForce()
    {
        return _joint.currentForce;
    }

    public void connectBodyPart(int targetHoldID)//float dis
    {
        if (_locker_body == null)
        {
            _locker_body = _bodyPartInfo.GetComponent<Rigidbody>();
        }

        if (current_hold_id != targetHoldID)
        {
            _locker_body.isKinematic = true;

            IsConnected = true;
            current_hold_id = targetHoldID;
        }

        //float maxDisToHolds = 0.5f;
        //if (holdContact.touchingHold && holdContact.hold_id == target_hold_id && dis <= maxDisToHolds)
        //{
        //    if (cHoldId != target_hold_id)
        //    {
        //        _locker_body.isKinematic = true;

        //        IsConnected = true;
        //        cHoldId = target_hold_id;
        //    }
        //}
        //else if ((cHoldId != target_hold_id && cHoldId >= 0) || (dis > maxDisToHolds && _locker_body.isKinematic))
        //{
        //    disconnectBodyPart();
        //}
    }

    public void disconnectBodyPart()
    {
        if (_locker_body == null)
        {
            _locker_body = _bodyPartInfo.GetComponent<Rigidbody>();
        }
        _locker_body.isKinematic = false;

        current_hold_id = -1;
        IsConnected = false;
    }

    public Rigidbody GetRigidBody()
    {
        if (_locker_body == null)
        {
            _locker_body = _bodyPartInfo.GetComponent<Rigidbody>();
        }
        return _locker_body;
    }
}
