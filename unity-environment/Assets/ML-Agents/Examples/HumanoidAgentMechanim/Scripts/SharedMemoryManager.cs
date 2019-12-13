using System.Collections.Generic;
using UnityEngine;
using System.Net.Sockets;
using System.Text;

namespace Assets.ML_Agents.Examples.SharedAssets.Scripts
{
    public class SharedMemoryManager
    {
        enum _case_study
        {
            spline_full_body_feature, spline_attached_feet, spline_target_share, full_body_one_step_action, arm_state_action_one_step
        };

        public List<ClimberMechanimRig.ClimberBodyState> savedStates = new List<ClimberMechanimRig.ClimberBodyState>(); // storing all states in the trajectory manager
        int currentFreeIndex = 0;

        public int GetNextFreeSlotIdx()
        {
            if (currentFreeIndex < savedStates.Count)
            {
                currentFreeIndex++;
                return currentFreeIndex - 1;
            }
            savedStates.Add(new ClimberMechanimRig.ClimberBodyState());
            currentFreeIndex = savedStates.Count;
            return savedStates.Count - 1; // return already a valid state
        }

        public void SetCurrentFreeIndex(int _nFreeIndex)
        {
            currentFreeIndex = _nFreeIndex;
        }

        public void GetHoldIds(int _memoryIdx, ref int[] _outHoldIds)
        {
            for (int i = 0; i < 4; i++)
                _outHoldIds[i] = savedStates[_memoryIdx].hold_bodies_ids[i];
            return;
        }

        public int GetCurrentHoldId(int _memoryIdx, int id)
        {
            return savedStates[_memoryIdx].hold_bodies_ids[id];
        }

        public void GetEndPointPoses(int _memoryIdx, ref List<Vector3> _outposes)
        {
            for (int i = 0; i < 4; i++)
                _outposes.Add(savedStates[_memoryIdx].end_bodies_pos[i]);
            return;
        }

        public Vector3 GetEndBonePosition(int _memoryIdx, int boneId)
        {
            return savedStates[_memoryIdx].end_bodies_pos[boneId];
        }

        //public Vector3 GetVel(int _memoryIdx, int boneId)
        //{
        //    return savedStates[_memoryIdx].Vel(boneId);
        //}

        public Vector3 GetHipPos(int _memoryIdx)
        {
            return savedStates[_memoryIdx].hipLocation;
        }

        //public Vector3 GetHipVel(int _memoryIdx)
        //{
        //    return savedStates[_memoryIdx].Vel(0);
        //}

        public bool SaveState(int _freeSlotIdx, int _fromSlot)
        {
            if (savedStates == null)
                return false;

            if (!(_freeSlotIdx >= 0 && _freeSlotIdx < savedStates.Count))
            {
                return false;
            }
            if (!(_fromSlot >= 0 && _fromSlot < savedStates.Count))
            {
                return false;
            }
            //savedStates[_freeSlotIdx].Copy(savedStates[_fromSlot]);

            return true; // successful saving
        }

        public void LoadStateFromList(int _freeSlotIdx, ref List<float> inList)
        {
            if (!(_freeSlotIdx >= 0 && _freeSlotIdx < savedStates.Count))
            {
                return;
            }
            //savedStates[_freeSlotIdx].LoadStateFromList(ref inList);
        }
        
        public void GetFeatureState(int stateIdx, ref List<float> outState, ref int[] tHolds, ref Vector3[] tHoldPoses, int[] boneIndices)
        {
            //_case_study _Study = _case_study.full_body_one_step_action;
            //switch (_Study)
            //{
            //    case _case_study.spline_full_body_feature:
            //        {
            //            Quaternion root_angle = savedStates[stateIdx].GetBoneAngle(0);
            //            Quaternion root_inverse = Quaternion.Inverse(root_angle);
            //            Vector3 root_pos = savedStates[stateIdx].GlobalPos(0);

            //            Vector3 v0 = savedStates[stateIdx].Vel(0);
            //            Vector3 av0 = savedStates[stateIdx].AVel(0);

            //            // gravity direction (3)
            //            Tools.PushStateFeature(ref outState, root_inverse * (new Vector3(0.0f, -1.0f, 0.0f)));

            //            // climber velocity and angular velocity (6)
            //            Tools.PushStateFeature(ref outState, root_inverse * v0);
            //            Tools.PushStateFeature(ref outState, root_inverse * av0);

            //            // humanoid full feature 14 * (4 + 3 + 3 + 3)
            //            for (int i = 1; i < savedStates[stateIdx].GetBoneCount(); i++)
            //            {
            //                Tools.PushStateFeature(ref outState, savedStates[stateIdx].GetBoneAngle(i));
            //                Tools.PushStateFeature(ref outState, savedStates[stateIdx].GetBonePos(i));

            //                Tools.PushStateFeature(ref outState, root_inverse * savedStates[stateIdx].Vel(i));
            //                Tools.PushStateFeature(ref outState, root_inverse * savedStates[stateIdx].AVel(i));
            //            }

            //            // whether climber hands/feet are connected or not (4)
            //            for (int i = 0; i < 4; i++)
            //            {
            //                float _C = savedStates[stateIdx].hold_bodies_ids[i] >= 0 ? 0f : -1f;
            //                Tools.PushStateFeature(ref outState, _C);
            //            }
            //            // relative current hands/feet location (12)
            //            for (int i = 0; i < 4; i++)
            //            {
            //                Vector3 pos_relative_root = savedStates[stateIdx].end_bodies_pos[i] - root_pos;
            //                Tools.PushStateFeature(ref outState, root_inverse * pos_relative_root);
            //            }

            //            // climber target holds ids (4)
            //            for (int i = 0; i < 4; i++)
            //            {
            //                float _C = tHolds[i] >= 0 ? 0f : -1f;
            //                Tools.PushStateFeature(ref outState, _C);
            //            }
            //            // relative location of target holds (12)
            //            for (int i = 0; i < 4; i++)
            //            {
            //                Vector3 pos_relative_root = tHoldPoses[i] - root_pos;
            //                Tools.PushStateFeature(ref outState, root_inverse * pos_relative_root);
            //            }

            //            // body distance to wall (15 * 2)
            //            for (int i = 0; i < savedStates[stateIdx].GetBoneCount(); i++)
            //            {
            //                Tools.PushStateFeature(ref outState, 1.525f - savedStates[stateIdx].GlobalPos(i)[2]);
            //                if (savedStates[stateIdx].GlobalPos(i)[1] < 2.0f)
            //                    Tools.PushStateFeature(ref outState, savedStates[stateIdx].GlobalPos(i)[1]);
            //                else
            //                    Tools.PushStateFeature(ref outState, 2.0f);
            //            }

            //            // initialization of the splines
            //            for (int i = 0; i < savedStates[stateIdx].spline.Count; i++)
            //            {
            //                Tools.PushStateFeature(ref outState, savedStates[stateIdx].spline[i].currentValue);
            //                Tools.PushStateFeature(ref outState, savedStates[stateIdx].spline[i].currentDerivativeValue);
            //            }
            //            // 3 + 6 + 14 * 13 + 4 + 12 + 4 + 12 + 15 * 2 + 30 * 2 = 313
            //        }
            //        break;
            //    case _case_study.spline_attached_feet:
            //        {
            //            Vector3 v0 = savedStates[stateIdx].Vel(0);
            //            Vector3 av0 = savedStates[stateIdx].AVel(0);

            //            Quaternion root_angle = savedStates[stateIdx].GetBoneAngle(0);
            //            Quaternion root_inverse = Quaternion.Inverse(root_angle);
            //            Vector3 root_pos = savedStates[stateIdx].GlobalPos(0);

            //            // gravity direction (3)
            //            Tools.PushStateFeature(ref outState, root_inverse * (new Vector3(0.0f, -1.0f, 0.0f)));

            //            // climber velocity and angular velocity (6)
            //            Tools.PushStateFeature(ref outState, root_inverse * v0);
            //            Tools.PushStateFeature(ref outState, root_inverse * av0);

            //            // climber current spline target angles and angle rates (30)
            //            for (int i = 0; i < savedStates[stateIdx].spline.Count; i++)
            //            {
            //                Tools.PushStateFeature(ref outState, savedStates[stateIdx].spline[i].currentValue);
            //                Tools.PushStateFeature(ref outState, savedStates[stateIdx].spline[i].currentDerivativeValue);
            //            }
            //            //// boneIndices.Length = 10
            //            // climber posture && relative velocity (10 * (3) = 30)
            //            for (int i = 0; i < boneIndices.Length; i++)
            //            {
            //                //Vector3 pos_relative_root = savedStates[stateIdx].GlobalPos(boneIndices[i]) - root_pos;
            //                //Tools.PushStateFeature(ref outState, savedStates[stateIdx].GetBoneAngle(boneIndices[i]));
            //                //Tools.PushStateFeature(ref outState, root_inverse * pos_relative_root);
            //                Tools.PushStateFeature(ref outState, root_inverse * savedStates[stateIdx].Vel(boneIndices[i]));
            //            }

            //            // whether climber hands/feet are connected or not (4)
            //            for (int i = 0; i < 4; i++)
            //            {
            //                float _C = savedStates[stateIdx].hold_bodies_ids[i] >= 0 ? 0f : -1f;
            //                Tools.PushStateFeature(ref outState, _C);
            //            }
            //            // relative current hands/feet location (12)
            //            for (int i = 0; i < 4; i++)
            //            {
            //                Vector3 pos_relative_root = savedStates[stateIdx].end_bodies_pos[i] - root_pos;
            //                Tools.PushStateFeature(ref outState, root_inverse * pos_relative_root);
            //            }

            //            // climber target holds ids (4)
            //            for (int i = 0; i < 4; i++)
            //            {
            //                float _C = tHolds[i] >= 0 ? 0f : -1f;
            //                Tools.PushStateFeature(ref outState, _C);
            //            }
            //            // relative location of target holds (12)
            //            for (int i = 0; i < 4; i++)
            //            {
            //                Vector3 pos_relative_root = tHoldPoses[i] - root_pos;
            //                Tools.PushStateFeature(ref outState, root_inverse * pos_relative_root);
            //            }

            //            // body distance to wall (15)
            //            for (int i = 0; i < savedStates[stateIdx].GetBoneCount(); i++)
            //            {
            //                Tools.PushStateFeature(ref outState, 1.525f - savedStates[stateIdx].GlobalPos(i)[2]);
            //            }
            //            // 3 + 6 + 100 + 4 + 12 + 4 + 12 + 15 -100 - 15 = 156 -100 + 30 + 30= 116
            //        }
            //        break;
            //    case _case_study.spline_target_share:
            //        {
            //            Vector3 v0 = savedStates[stateIdx].Vel(0);
            //            Vector3 av0 = savedStates[stateIdx].AVel(0);
            //            Vector3 root_pos = savedStates[stateIdx].GlobalPos(0);

            //            // climber velocity and angular velocity (6)
            //            Tools.PushStateFeature(ref outState, v0);
            //            Tools.PushStateFeature(ref outState, av0);

            //            // relative current hands/feet location (12)
            //            for (int i = 0; i < 4; i++)
            //            {
            //                Vector3 pos_relative_root = savedStates[stateIdx].end_bodies_pos[i] - root_pos;
            //                Tools.PushStateFeature(ref outState, pos_relative_root);
            //            }

            //            // climber current spline target angles and angle rates (30 * 2)
            //            for (int i = 0; i < savedStates[stateIdx].spline.Count; i++)
            //            {
            //                Tools.PushStateFeature(ref outState, savedStates[stateIdx].spline[i].currentValue);
            //                Tools.PushStateFeature(ref outState, savedStates[stateIdx].spline[i].currentDerivativeValue);
            //            }

            //            // climber target holds ids (4)
            //            bool flag_added = false;
            //            for (int i = 0; i < 4; i++)
            //            {
            //                if (savedStates[stateIdx].hold_bodies_ids[i] != tHolds[i] && tHolds[i] > -1)
            //                {
            //                    Tools.PushStateFeature(ref outState, (float)(i));
            //                    Vector3 pos_relative_root = tHoldPoses[i] - root_pos;
            //                    Tools.PushStateFeature(ref outState, pos_relative_root);
            //                    flag_added = true;
            //                    break;
            //                }
            //            }
            //            if (!flag_added)
            //            {
            //                Tools.PushStateFeature(ref outState, -1.0f);
            //                Vector3 pos_relative_root = new Vector3(-1.0f, -1.0f, -1.0f) - root_pos;
            //                Tools.PushStateFeature(ref outState, pos_relative_root);
            //            }
            //        }
            //        break;
            //    case _case_study.full_body_one_step_action:
            //        {
            //            Vector3 gravity_dir = new Vector3(0.0f, -1.0f, 0.0f);

            //            Quaternion root_angle = savedStates[stateIdx].GetBoneAngle(0);
            //            Vector3 root_pos = savedStates[stateIdx].GlobalPos(0);

            //            Vector3 v0 = savedStates[stateIdx].Vel(0);
            //            Vector3 av0 = savedStates[stateIdx].AVel(0);

            //            // gravity direction (3)
            //            Tools.PushStateFeature(ref outState, root_angle * gravity_dir);

            //            // climber velocity and angular velocity (6)
            //            Tools.PushStateFeature(ref outState, v0);
            //            Tools.PushStateFeature(ref outState, av0);

            //            // humanoid full feature 14 * (3 + 3 + 3 + 3) = 168
            //            for (int i = 1; i < savedStates[stateIdx].GetBoneCount(); i++)
            //            {
            //                Tools.PushStateFeature(ref outState, savedStates[stateIdx].GetBoneAngle(i) * gravity_dir);
            //                Tools.PushStateFeature(ref outState, savedStates[stateIdx].GetBonePos(i));
            //                Tools.PushStateFeature(ref outState, savedStates[stateIdx].Vel(i));
            //                Tools.PushStateFeature(ref outState, savedStates[stateIdx].AVel(i));
            //            }

            //            // whether climber hands/feet are connected or not (4)
            //            for (int i = 0; i < 4; i++)
            //            {
            //                float _C = savedStates[stateIdx].hold_bodies_ids[i] >= 0 ? 0f : -1f;
            //                Tools.PushStateFeature(ref outState, _C);
            //            }
            //            // relative current hands/feet location (12)
            //            for (int i = 0; i < 4; i++)
            //            {
            //                Vector3 pos_relative_root = savedStates[stateIdx].end_bodies_pos[i] - root_pos;
            //                Tools.PushStateFeature(ref outState, pos_relative_root);
            //            }

            //            // climber target holds ids (4)
            //            for (int i = 0; i < 4; i++)
            //            {
            //                float _C = tHolds[i] >= 0 ? 0f : -1f;
            //                Tools.PushStateFeature(ref outState, _C);
            //            }
            //            // relative location of target holds (12)
            //            for (int i = 0; i < 4; i++)
            //            {
            //                Vector3 pos_relative_root = tHoldPoses[i] - root_pos;
            //                Tools.PushStateFeature(ref outState, pos_relative_root);
            //            }

            //            // body distance to wall (15 * 2)
            //            for (int i = 0; i < savedStates[stateIdx].GetBoneCount(); i++)
            //            {
            //                Tools.PushStateFeature(ref outState, savedStates[stateIdx].disToWall[i]);
            //                Tools.PushStateFeature(ref outState, savedStates[stateIdx].disToFloor[i]);
            //            }
            //            // 3 + 6 + 14 * 12 + 4 + 12 + 4 + 12 + 15 * 2 = 239
            //        }
            //        break;
            //    case _case_study.arm_state_action_one_step:
            //        {

            //        }
            //        break;
            //    default:
            //        break;
            //}

            return;
        }

        public void GetState(int stateIdx, ref List<float> outState)
        {
            //savedStates[stateIdx].SaveStateToList(ref outState);
            //return;
        }

    };


}
