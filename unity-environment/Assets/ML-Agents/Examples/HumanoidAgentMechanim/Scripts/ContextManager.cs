using System.Collections.Generic;
using UnityEngine;
using Assets.ML_Agents.Examples.SharedAssets.Scripts;
using Assets.ML_Agents.Examples.ClimberScripts;
using UnityEditor;

public class ContextManager : MonoBehaviour
{
    //List<Vector3> _holdsPositions = new List<Vector3>();// not-biased positions
    List<Transform> _holdsTrasforms = new List<Transform>();// not-biased positions
    float wallZPos;
    float climberRadius;
    float climberHandHandDis;
    float climberLegLegDis;
    float LeftLegPosX;
    float RightLegPosX;
    float FootZ;
    int goal_hold_id = -1;
    List<List<int>> indices_around_hand = new List<List<int>>();
    List<List<int>> possible_hold_index_diff = null;
    HoldInfo.HoldType currentHoldType = HoldInfo.HoldType.Sphere;

    public GameObject agentGameObject;

    public HoldInfo.HoldType targetHoldType = HoldInfo.HoldType.Sphere;

    public int[] retNearHandIndex = { -1, -1 };

    public bool FlagCanRandomizeFromInterface = true;
    public bool RandomizeHolds = true;
    
    // Start is called before the first frame update
    void Start()
    {
        for (int i = 0; i < agentGameObject.transform.childCount; i++)
        {
            Transform child_transform = agentGameObject.transform.GetChild(i);
            if (child_transform.name.Contains("Hold"))
            {
                Vector3 pos = child_transform.localPosition;

                child_transform.GetComponent<HoldInfo>().holdId = _holdsTrasforms.Count;

                _holdsTrasforms.Add(child_transform);

                wallZPos = pos.z;
            }
        }
        ClimberMechanimRig rig = agentGameObject.GetComponent<ClimberMechanimRig>();
        // just to make sure the rig is initialized
        rig.Initialize();

        climberHandHandDis = (rig.GetEndBonePosition((int)LowLevelController.ControlledPoses.RightHand) 
                             - rig.GetEndBonePosition((int)LowLevelController.ControlledPoses.LeftHand)).magnitude;
        climberLegLegDis = 2.0f * (rig.GetEndBonePosition((int)LowLevelController.ControlledPoses.RightLeg) - rig.pos(HumanBodyBones.Hips)).magnitude;
        climberRadius = (rig.pos(HumanBodyBones.Chest) - rig.pos(HumanBodyBones.Hips)).magnitude + (climberHandHandDis / 2.0f) + (climberLegLegDis / 2.0f);
        LeftLegPosX = rig.GetEndBonePosition((int)LowLevelController.ControlledPoses.LeftLeg).x;
        RightLegPosX = rig.GetEndBonePosition((int)LowLevelController.ControlledPoses.RightLeg).x;
        FootZ = rig.GetEndBonePosition((int)LowLevelController.ControlledPoses.RightLeg).y;
        goal_hold_id = (int)(NumHolds() - 1);
    }

    // Update is called once per frame
    void Update()
    {
        if (RandomizeHolds && FlagCanRandomizeFromInterface)
        {
            RandomizeHoldPositions();
            RandomizeHolds = false;
        }

        if (currentHoldType != targetHoldType)
        {
            for (int h = 0; h < _holdsTrasforms.Count; h++)
                _holdsTrasforms[h].GetComponent<HoldInfo>().targetHoldType = targetHoldType;
            currentHoldType = targetHoldType;
        }
    }

    public void DeactiveRandomizeFromInterface()
    {
        FlagCanRandomizeFromInterface = false;
    }

    public void SetColorToHolds(int hold_id, Color _color)
    {
        _color.a = 0.25f;
        if (hold_id > -1)
            _holdsTrasforms[hold_id].gameObject.GetComponentInChildren<Renderer>().material.color = _color;
        return;
    }

    public int GetNumHolds()
    {
        return _holdsTrasforms.Count;
    }

    public float ConnectionThreshold { get; set; } = 0.2f;

    public Vector3 GetHoldGlobalPosition(int h)
    {
        if (h < 0 || h >= NumHolds())
            return new Vector3(0, 0, 0);
        return _holdsTrasforms[h].position;
    }

    public Vector3 GetHoldPosition(int h)
    {
        if (h < 0 || h >= NumHolds())
            return new Vector3(0, 0, 0);
        return _holdsTrasforms[h].localPosition;
    }

    public void SetHoldPosition(int h, Vector3 _pos)
    {
        if (h < NumHolds() && h >= 0)
        {
            _pos[2] = wallZPos; // hold on the wall!
            if (_pos[1] < 0.1f)
                _pos[1] = 0.1f;
            if (_pos[1] > 8.5f)
                _pos[1] = 8.5f;
            if (_pos[0] > 2.5f)
                _pos[0] = 2.5f;
            if (_pos[0] < -2.5f)
                _pos[0] = -2.5f;

            _holdsTrasforms[h].localPosition = _pos;

            //_holdsPositions[h] = _pos;
        }
    }

    public Quaternion GetHoldRotation(int h)
    {
        if (h < 0 || h >= NumHolds())
            return new Quaternion(0, 0, 0, 1.0f);
        return _holdsTrasforms[h].localRotation;
    }

    public void SetHoldRotation(int h, Quaternion _rotation)
    {
        if (h < NumHolds() && h >= 0)
        {
            _holdsTrasforms[h].localRotation = _rotation;
        }
    }

    public HoldInfo.HoldType GetHoldType(int h)
    {
        if (h < 0 || h >= NumHolds())
            return HoldInfo.HoldType.Sphere;
        return _holdsTrasforms[h].GetComponent<HoldInfo>().targetHoldType;
    }

    public void SetHoldType(int h, HoldInfo.HoldType _type)
    {
        if (h < NumHolds() && h >= 0)
        {
            _holdsTrasforms[h].GetComponent<HoldInfo>().targetHoldType = _type;
        }
    }

    public int NumHolds()
    {
        return _holdsTrasforms.Count;
    }

    public void RandomizeHoldPositions()
    {
        float _angle = UnityEngine.Random.Range(50.0f, 130.0f);
        _angle = _angle * Mathf.Deg2Rad;

        // there is alwayse the same amount of holds but with randomized hold positions
        float startPosX = LeftLegPosX;
        float _dir_sign = Mathf.Sign(Mathf.Cos(_angle));

        retNearHandIndex[0] = retNearHandIndex[1] = -1;

        if (_dir_sign < 0)
        {
            startPosX = RightLegPosX;
        }

        float cHeightZ = FootZ;
        int index = 0;
        int des_row = 5;
        int des_col = 2;
        int miss_row = 50;
        int des_num_holds = des_col * (des_row - 1);

        // create holds in the direction of _dir from _angle starting from startPos
        Vector3 _dir = new Vector3(Mathf.Cos(_angle), Mathf.Sin(_angle), 0.0f);
        int index_on_line = 0;
        while (index < des_num_holds)
        {
            float row = _dir.y * index_on_line;
            float col = _dir.x * index_on_line;
            for (int i = 0; i < 2; i++)
            {
                float r1 = UnityEngine.Random.Range(0.0f, 1.0f);
                float r2 = UnityEngine.Random.Range(0.0f, 1.0f);

                float nZ = cHeightZ + row * (climberRadius / 2.0f) + (climberRadius / 5.0f) * r2;
                float nX = startPosX + col * (climberHandHandDis / 2.0f) - (climberHandHandDis / 2.1f) + (2.0f * r1 * (climberHandHandDis / 2.1f));
                Vector3 cPos = new Vector3(nX, nZ, wallZPos);

                // now some-times we have crossing hands! 
                ///////(TODO: in case you want to remove it, should check nX in a row; the min one should be set for left hand (idx = 0), the max one is for right hand (idx = 1))
                if (index_on_line == 1)
                {
                    if (col > 0)
                        retNearHandIndex[i] = index;
                    else
                        retNearHandIndex[1 - i] = index;
                }

                if (index_on_line != miss_row)
                {
                    SetHoldPosition(index, cPos);
                    index++;
                }
            }
            index_on_line++;
        }

        goal_hold_id = (int)(NumHolds() - 1);

        FillAroundHandHoldIndices();

        return;
    }

    public float GetClimberRadius()
    {
        return climberRadius;
    }
    
    // given a hand hold, we will return a stance sample in a uniform distribution
    public int[] GetRandomStanceAroundHandHold(int given_hold_id, int[] current_hold_ids, int _diff)
    {
        const int itr_index_diff_size = 4;

        int[] out_sample = { -1, -1, -1, -1 };

        if (possible_hold_index_diff == null)
        {
            possible_hold_index_diff = new List<List<int>>(itr_index_diff_size);
            for (int i = 0; i < itr_index_diff_size; i++)
            {
                possible_hold_index_diff.Add(new List<int>());
            }
        }

        for (int i = 0; i < itr_index_diff_size; i++)
        {
            possible_hold_index_diff[i].Clear();
        }
        //set right hand possible hold ids
        MyTools.AddToSetIDs(given_hold_id, possible_hold_index_diff[3]);

        Vector3 center_hold = GetHoldPosition(given_hold_id);
        for (int j = 0; j < indices_around_hand[given_hold_id].Count; j++)
        {
            int hold_id_j = indices_around_hand[given_hold_id][j];
            Vector3 hold_j = GetHoldPosition(hold_id_j);

            Vector3 _displacement = (hold_j - center_hold);
            float _dis = _displacement.magnitude;

            if (_dis < climberHandHandDis)
            {
                MyTools.AddToSetIDs(hold_id_j, possible_hold_index_diff[2]);
            }

            MyTools.AddToSetIDs(hold_id_j, possible_hold_index_diff[0]);
            MyTools.AddToSetIDs(hold_id_j, possible_hold_index_diff[1]);
        }

        for (int i = 0; i < itr_index_diff_size; i++)
        {
            MyTools.AddToSetIDs(-1, possible_hold_index_diff[i]);
        }

        bool flag_continue = true;
        int counter = 0;
        int max_counter = possible_hold_index_diff[3].Count * possible_hold_index_diff[2].Count * possible_hold_index_diff[1].Count * possible_hold_index_diff[0].Count;
        while (flag_continue)
        {
            // create sample n
            int[] indices = { 0, 1, 2, 3 };
            MyTools.Shuffle(ref indices);

            for (int i = 0; i < 4; i++)
            {
                out_sample[i] = current_hold_ids[i];
            }

            for (int m = 0; m < _diff; m++)
            {
                int i = indices[m];
                List<int> possible_hold_diff_i = possible_hold_index_diff[i];

                int itr_index_diff_i = UnityEngine.Random.Range(0, possible_hold_index_diff[i].Count);
                if (itr_index_diff_i >= possible_hold_index_diff[i].Count - 1)
                    itr_index_diff_i = possible_hold_index_diff[i].Count - 1;
                out_sample[i] = possible_hold_diff_i[itr_index_diff_i];
            }

            if (out_sample[3] == -1)
                out_sample[2] = given_hold_id;

            ///////////////////////////////////////////////////////////////
            // prior for adding sample_n to the list of possible samples //
            ///////////////////////////////////////////////////////////////
            if (IsValidStanceSample(out_sample))
            {
                flag_continue = false;
                return out_sample;
            }

            counter++;
            if (counter > max_counter)
            {
                for (int i = 0; i < itr_index_diff_size; i++)
                {
                    out_sample[i] = current_hold_ids[i];
                }
                flag_continue = false;
                return out_sample;
            }
        }
        return out_sample;
    }

    // given a hand hold, we will return a stance sample in a uniform distribution
    public int[] GetRandomStanceAroundHandHold(int given_hold_id)
    {
        const int itr_index_diff_size = 4;

        int[] out_sample = { -1, -1, -1, -1 };

        if (possible_hold_index_diff == null)
        {
            possible_hold_index_diff = new List<List<int>>(itr_index_diff_size);
            for (int i = 0; i < itr_index_diff_size; i++)
            {
                possible_hold_index_diff.Add(new List<int>());
            }
        }

        for (int i = 0; i < itr_index_diff_size; i++)
        {
            possible_hold_index_diff[i].Clear();
        }
        //set right hand possible hold ids
        MyTools.AddToSetIDs(given_hold_id, possible_hold_index_diff[3]);

        Vector3 center_hold = GetHoldPosition(given_hold_id);
        for (int j = 0; j < indices_around_hand[given_hold_id].Count; j++)
        {
            int hold_id_j = indices_around_hand[given_hold_id][j];
            Vector3 hold_j = GetHoldPosition(hold_id_j);

            Vector3 _displacement = (hold_j - center_hold);
            float _dis = _displacement.magnitude;

            if (_dis < climberHandHandDis)
            {
                MyTools.AddToSetIDs(hold_id_j, possible_hold_index_diff[2]);
            }

            MyTools.AddToSetIDs(hold_id_j, possible_hold_index_diff[0]);
            MyTools.AddToSetIDs(hold_id_j, possible_hold_index_diff[1]);
        }

        for (int i = 0; i < itr_index_diff_size; i++)
        {
            MyTools.AddToSetIDs(-1, possible_hold_index_diff[i]);
        }

        bool flag_continue = true;
        int counter = 0;
        int max_counter = possible_hold_index_diff[3].Count * possible_hold_index_diff[2].Count * possible_hold_index_diff[1].Count * possible_hold_index_diff[0].Count;
        while (flag_continue)
        {
            // create sample n
            int[] indices = { 0, 1, 2, 3 };
            MyTools.Shuffle(ref indices);

            for (int m = 0; m < 4; m++)
            {
                int i = indices[m];
                List<int> possible_hold_diff_i = possible_hold_index_diff[i];

                int itr_index_diff_i = UnityEngine.Random.Range(0, possible_hold_index_diff[i].Count);
                if (itr_index_diff_i >= possible_hold_index_diff[i].Count - 1)
                    itr_index_diff_i = possible_hold_index_diff[i].Count - 1;
                out_sample[i] = possible_hold_diff_i[itr_index_diff_i];
            }

            if (out_sample[3] == -1)
                out_sample[2] = given_hold_id;

            ///////////////////////////////////////////////////////////////
            // prior for adding sample_n to the list of possible samples //
            ///////////////////////////////////////////////////////////////
            if (IsValidStanceSample(out_sample))
            {
                flag_continue = false;
                return out_sample;
            }

            counter++;
            if (counter > max_counter)
            {
                flag_continue = false;
                return out_sample;
            }
        }
        return out_sample;
    }

    public Vector3 GetHoldStancePosFrom(int[] _hold_ids, ref List<Vector3> _hold_points, ref float mSize)
    {
        Vector3 midPoint = new Vector3(0.0f, 0.0f, 0.0f);
        mSize = 0.0f;
        _hold_points.Clear();
        for (int i = 0; i < _hold_ids.Length; i++)
        {
            if (_hold_ids[i] != -1)
            {
                _hold_points.Add(GetHoldPosition(_hold_ids[i]));
                midPoint += _hold_points[i];
                mSize++;
            }
            else
                _hold_points.Add(new Vector3(0, 0, 0));
        }

        if (mSize > 0)
        {
            midPoint = midPoint / mSize;
        }
        else//we are on the ground
        {
            midPoint = new Vector3(0.0f, 1.1f, 0.85f);
        }
        return midPoint;
    }

    public Vector3 GetMidHoldStancePos(int[] _hold_ids)
    {
        Vector3 midPoint = new Vector3(0.0f, 0.0f, 0.0f);
        float mSize = 0.0f;
        for (int i = 0; i < _hold_ids.Length; i++)
        {
            if (_hold_ids[i] != -1)
            {
                Vector3 p = GetHoldPosition(_hold_ids[i]);
                midPoint += p;
                mSize++;
            }
        }

        if (mSize > 0)
        {
            midPoint = midPoint / mSize;
        }
        else//we are on the ground
        {
            midPoint = new Vector3(0.0f, 1.1f, 0.85f);
        }
        return midPoint;
    }

    public Vector3 GetGoalHoldPosition()
    {
        return _holdsTrasforms[goal_hold_id].localPosition;
    }

    void FillAroundHandHoldIndices()
    {
        // num of holds are alwayse the same!
        if (indices_around_hand.Count > 0)
        {
            for (int k = 0; k < NumHolds(); k++)
            {
                indices_around_hand[k].Clear();
            }
            indices_around_hand.Clear();
        }

        float max_body_radius = climberRadius;
        for (int k = 0; k < NumHolds(); k++)
        {
            Vector3 dHoldPos = GetHoldPosition(k);
            List<int> hand_holds_ids = new List<int>();
            GetHoldsInRadius(dHoldPos, max_body_radius, ref hand_holds_ids);
            indices_around_hand.Add(hand_holds_ids);
        }

        return;
    }

    void GetHoldsInRadius(Vector3 dP, float r, ref List<int> ret_holds_ids)
    {
        for (int i = 0; i < NumHolds(); i++)
        {
            Vector3 hold_i = GetHoldPosition(i);

            float cDis = (hold_i - dP).magnitude;
            if (cDis < r)
            {
                ret_holds_ids.Add(i);
            }
        }
    }

    bool IsValidStanceSample(int[] sample_stance)
    {
        if (sample_stance[2] == -1 && sample_stance[3] == -1)
            return false;

        List<Vector3> sample_hold_points = new List<Vector3>();
        float mSize = 0.0f;
        GetHoldStancePosFrom(sample_stance, ref sample_hold_points, ref mSize);

        float dis_feet = 0.0f;
        if (sample_stance[0] != -1 && sample_stance[1] != -1)
        {
            dis_feet = (sample_hold_points[0] - sample_hold_points[1]).magnitude;
        }
        if (dis_feet > climberLegLegDis)
            return false;

        float dis_hands = 0.0f;
        if (sample_stance[2] != -1 && sample_stance[3] != -1)
        {
            dis_hands = (sample_hold_points[2] - sample_hold_points[3]).magnitude;
        }
        if (dis_hands > climberHandHandDis)
            return false;

        float dis_hand_feet = 0.0f;
        for (int f = 0; f < 2; f++)
        {
            if (sample_stance[f] != -1)
            {
                for (int h = 2; h < 4; h++)
                {
                    if (sample_stance[h] != -1)
                    {
                        dis_hand_feet = (sample_hold_points[f] - sample_hold_points[h]).magnitude;
                        if (dis_hand_feet > climberRadius)
                            return false;
                    }
                }
            }
        }

        // if both feet are higher than max hight hand
        float max_hight_yHand = float.MinValue;
        if (sample_stance[2] != -1)
            max_hight_yHand = Mathf.Max(max_hight_yHand, sample_hold_points[2].y);
        if (sample_stance[3] != -1)
            max_hight_yHand = Mathf.Max(max_hight_yHand, sample_hold_points[3].y);

        if (sample_stance[0] != -1 || sample_stance[1] != -1)
        {
            float d_feet_threshold = 0.5f;
            if (sample_stance[0] != -1)
            {
                if (sample_hold_points[0].y >= max_hight_yHand + d_feet_threshold)
                    return false;
            }
            if (sample_stance[1] != -1)
            {
                if (sample_hold_points[1].y >= max_hight_yHand + d_feet_threshold)
                    return false;
            }
            if (sample_stance[0] != -1 && sample_stance[1] != -1)
            {
                if (sample_hold_points[0].y >= max_hight_yHand && sample_hold_points[1].y >= max_hight_yHand)
                    return false;
            }
        }

        return true;
    }
   
}

[CustomEditor(typeof(ContextManager))]
public class MyScriptEditor : Editor
{
    public override void OnInspectorGUI()
    {
        var myScript = target as ContextManager;

        myScript.agentGameObject = EditorGUILayout.ObjectField("Agent Game Object", myScript.agentGameObject, typeof(GameObject), true) as GameObject;

        myScript.targetHoldType = (HoldInfo.HoldType)EditorGUILayout.EnumPopup("Primitive Holds:", myScript.targetHoldType);

        //myScript.retNearHandIndex = { -1, -1 };

        //myScript.FlagCanRandomizeFromInterface = GUILayout.Toggle(myScript.FlagCanRandomizeFromInterface, "EnableRandomize");

        if (myScript.FlagCanRandomizeFromInterface)
            myScript.RandomizeHolds = EditorGUILayout.Toggle("FlagRandomize", myScript.RandomizeHolds);

    }
}