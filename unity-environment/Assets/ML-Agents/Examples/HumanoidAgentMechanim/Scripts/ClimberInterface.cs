using System.Collections.Generic;
using UnityEngine;
using Assets.ML_Agents.Examples.SharedAssets.Scripts;
using System.IO;

namespace Assets.ML_Agents.Examples.ClimberScripts
{
    public class ClimberInterface
    {
        public enum UserCommandType { UserForwardSimulate = 0, UserNone = 1 };

        public class UserTrajectoryPoints
        {
            public int cStateID = -1;

            public List<int> _fromToStates = new List<int>();

            public int[] target_ids = { -1, -1, -1, -1 };
        }

        Vector3 lookAtPos = new Vector3();
        Vector3 camera_pos = new Vector3(10f, -Mathf.PI / 2.0f, 0.0f);
        Vector3 lastMouseClicked = new Vector3();
        int current_select_state = 0;
        int selected_limb = -1;
        int selected_hold = -1;
        public UserCommandType current_command_type = UserCommandType.UserNone;

        List<UserTrajectoryPoints> user_trajectory_points = new List<UserTrajectoryPoints>();
        Vector2Int cStepAnimation = new Vector2Int(0, 0);

        ClimberMechanimRig controlRig;
        SharedMemoryManager memoryManager;
        ContextManager masterContext;

        bool isTaskCompleted = true;

        int[] init_hold_ids = { -1, -1, -1, -1 };
        int[] target_hold_ids = { -1, -1, -1, -1 };
        int total_states_saved = 0;

        // variables for playing animations
        float current_scroll_value = 0.0f;
        bool flag_play_loop = true;
        float time_diff_happens = 0.0f;

        public ClimberInterface(GameObject iGameInstant, SharedMemoryManager iMemoryManager)
        {
            controlRig = iGameInstant.GetComponentsInChildren<ClimberMechanimRig>()[0];

            masterContext = iGameInstant.GetComponent<ContextManager>();

            memoryManager = iMemoryManager;

            AddTrajectoryPoint(null);
        }

        public bool LoadCurrentState()
        {
            return controlRig.LoadState(user_trajectory_points[user_trajectory_points.Count - 1].cStateID);
        }

        public void UpdateTrajectoryPoints(bool isOptimizationDone, LowLevelController.SamplingTrajectoryStr bestSampleTrajectory)
        {
            if (isOptimizationDone)
            {
                AddTrajectoryPoint(bestSampleTrajectory);
                isTaskCompleted = true;
                current_command_type = UserCommandType.UserNone;
            }

            return;
        }

        void CheckForUserInterrupt(LowLevelController.SamplingTrajectoryStr bestSampleTrajectory)
        {
            int limb_id = selected_limb;
            int hold_id = selected_hold;
            if (limb_id < 0)
                return;

            if (user_trajectory_points[user_trajectory_points.Count - 1].target_ids[limb_id] == hold_id)
                return;

            if (current_command_type == UserCommandType.UserForwardSimulate)
            {
                UpdateTrajectoryPoints(true, bestSampleTrajectory);
                current_command_type = UserCommandType.UserNone;
            }
            return;
        }

        public void UseInterface(ref SamplingHighLevelPlan _samplePlan,
            LowLevelController.SamplingTrajectoryStr bestSampleTrajectory,
            bool play_animation)
        {
            Vector3 nCameraPos = lookAtPos + camera_pos[0] * (new Vector3(Mathf.Cos(camera_pos[1]), Mathf.Sin(camera_pos[2]), Mathf.Sin(camera_pos[1]))).normalized;
            nCameraPos.y = Mathf.Max(0f, nCameraPos.y);

            Camera.main.transform.position = nCameraPos;
            Camera.main.transform.LookAt(lookAtPos, Vector3.up);

            Transform hips = controlRig.GetBoneTransform(HumanBodyBones.Hips);
            lookAtPos = 0.95f * lookAtPos + 0.05f * hips.position;

            if (Input.GetMouseButton(1))
            {
                Vector3 delta = Input.mousePosition - lastMouseClicked;

                camera_pos[1] += delta[0] / 100.0f;
                camera_pos[2] += delta[1] / 100.0f;

                if (camera_pos[2] < -Mathf.PI / 2.0f)
                    camera_pos[2] = -Mathf.PI / 2.0f;
                if (camera_pos[2] > Mathf.PI / 2.0f)
                    camera_pos[2] = Mathf.PI / 2.0f;

                if (camera_pos[1] < -Mathf.PI)
                    camera_pos[1] = -Mathf.PI;
                if (camera_pos[1] > 0)
                    camera_pos[1] = 0;
            }
            lastMouseClicked = Input.mousePosition;

            float mouse_scroll_value = Input.GetAxis("Mouse ScrollWheel");
            if (Mathf.Abs(mouse_scroll_value) > 0)
            {
                camera_pos[0] += -3.0f * mouse_scroll_value;
            }

            if (play_animation)
            {
                return;
            }

            RaycastHit hitInfo = new RaycastHit();
            bool hit = Physics.Raycast(Camera.main.ScreenPointToRay(Input.mousePosition), out hitInfo);
            if (hit)
            {
                if (hitInfo.transform.parent != null)
                {
                    if (hitInfo.transform.parent.name != "Environment")
                    {
                        if (hitInfo.transform.parent.parent != null && hitInfo.transform.parent.parent.name != "Environment")
                            return;
                    }
                }

                if (current_select_state == 0)
                {
                    if (hitInfo.transform.gameObject.name == "RHConnector" || hitInfo.transform.gameObject.name == "hand_R")
                    {
                        selected_limb = 3;
                    }
                    else if (hitInfo.transform.gameObject.name == "LHConnector" || hitInfo.transform.gameObject.name == "hand_L")
                    {
                        selected_limb = 2;
                    }
                    else if (hitInfo.transform.gameObject.name == "RLConnector" || hitInfo.transform.gameObject.name == "footR")
                    {
                        selected_limb = 1;
                    }
                    else if (hitInfo.transform.gameObject.name == "LLConnector" || hitInfo.transform.gameObject.name == "footL")
                    {
                        selected_limb = 0;
                    }
                    else
                    {
                        selected_limb = -1;
                    }
                }
                else
                {
                    //UnityEngine.Debug.Log(hitInfo.transform.gameObject.tag);
                    if (hitInfo.transform.gameObject.tag == "Hold")
                    {
                        HoldInfo cHoldInfo = hitInfo.transform.gameObject.GetComponent<HoldInfo>();
                        selected_hold = cHoldInfo.holdId;
                    }
                    else
                    {
                        selected_hold = -1;
                    }
                }
            }

            if (Input.GetMouseButton(0))
            {
                if (selected_limb > -1)
                {
                    current_select_state = 1;
                }
            }

            if (Input.GetMouseButtonUp(0))
            {
                CheckForUserInterrupt(bestSampleTrajectory);
                current_select_state = 0;
                if (selected_limb > -1)
                {
                    user_trajectory_points[user_trajectory_points.Count - 1].target_ids[selected_limb] = selected_hold;
                    selected_hold = -1;
                }
            }

            for (int h = 0; h < masterContext.GetNumHolds(); h++)
            {
                masterContext.SetColorToHolds(h, Color.yellow);
            }

            for (int b = 0; b < 4; b++)
            {
                if (user_trajectory_points[user_trajectory_points.Count - 1].target_ids[b] == controlRig.GetCurrentHoldBodyID(b))
                {
                    controlRig.SetColorToLimb(b, Color.black);
                }
                else
                {
                    controlRig.SetColorToLimb(b, Color.green);
                    if (user_trajectory_points[user_trajectory_points.Count - 1].target_ids[b] > -1)
                    {
                        masterContext.SetColorToHolds(user_trajectory_points[user_trajectory_points.Count - 1].target_ids[b], Color.green);
                        UnityEngine.Debug.DrawLine(controlRig.GetEndBoneGlobalPosition(b),
                            masterContext.GetHoldGlobalPosition(user_trajectory_points[user_trajectory_points.Count - 1].target_ids[b]), Color.black);
                    }
                }
            }

            if (selected_limb > -1)
            {
                controlRig.SetColorToLimb(selected_limb, Color.red);
            }

            if (selected_hold > -1)
            {
                masterContext.SetColorToHolds(selected_hold, Color.red);
            }

            if (Input.GetKeyDown(KeyCode.Return))
            {
                current_command_type = UserCommandType.UserForwardSimulate;

                isTaskCompleted = false;
                for (int b = 0; b < 4; b++)
                {
                    init_hold_ids[b] = controlRig.GetCurrentHoldBodyID(b);
                    target_hold_ids[b] = user_trajectory_points[user_trajectory_points.Count - 1].target_ids[b];
                }

                UpdateSampleHighLevelPlan(ref _samplePlan);
            }

            if (Input.GetKeyDown(KeyCode.Backspace))
            {
                RemoveLastTrajectoryPoint();
            }

            return;
        }

        public void SaveToFile()
        {
            StreamWriter streamWriter = new StreamWriter("user_scene_trajectory.txt", false);

            // save scene
            int numHolds = masterContext.GetNumHolds();
            streamWriter.WriteLine(numHolds.ToString());
            for (int h = 0; h < numHolds; h++)
            {
                streamWriter.WriteLine(masterContext.SaveHoldInfoToString(h));
            }

            int num_trajectory_points = user_trajectory_points.Count;
            streamWriter.WriteLine(num_trajectory_points.ToString());

            // now write num_trajectory_points trajectory points
            for (int i = 0; i < num_trajectory_points; i++)
            {
                // start with head line (num from-to states, target hold ids)
                string head_line = user_trajectory_points[i]._fromToStates.Count.ToString() + ","
                                 + user_trajectory_points[i].target_ids[0].ToString() + ","
                                 + user_trajectory_points[i].target_ids[1].ToString() + ","
                                 + user_trajectory_points[i].target_ids[2].ToString() + ","
                                 + user_trajectory_points[i].target_ids[3].ToString();
                streamWriter.WriteLine(head_line);

                // save current state
                streamWriter.WriteLine(
                    MyTools.ParseArrFloatInToString(
                        memoryManager.SaveStateToFloatArr(user_trajectory_points[i].cStateID),
                        ','));
                for (int s = 0; s < user_trajectory_points[i]._fromToStates.Count; s++)
                {
                    streamWriter.WriteLine(
                    MyTools.ParseArrFloatInToString(
                        memoryManager.SaveStateToFloatArr(user_trajectory_points[i]._fromToStates[s]),
                        ','));
                }
            }

            streamWriter.Flush();
            streamWriter.Close();
            return;
        }

        public void LoadFromFile()
        {
            total_states_saved = 0;

            StreamReader streamReader = new StreamReader("user_scene_trajectory.txt");

            // save scene
            int numHolds = masterContext.GetNumHolds();
            string line = streamReader.ReadLine();
            for (int h = 0; h < numHolds; h++)
            {
                masterContext.LoadHoldInfoToString(h, streamReader.ReadLine());
            }

            int num_trajectory_points = (int)(float.Parse(streamReader.ReadLine()));

            /////////////////////////////// TODO: replace this with something else, not good for memory management
            while (user_trajectory_points.Count > num_trajectory_points)
            {
                user_trajectory_points.RemoveAt(user_trajectory_points.Count - 1);
            }

            List<float> outArr = new List<float>();
            int c_index = 0;
            // now load num_trajectory_points trajectory points
            for (int i = 0; i < num_trajectory_points; i++)
            {
                if (i >= user_trajectory_points.Count)
                {
                    user_trajectory_points.Add(new UserTrajectoryPoints());
                }
                // start with head line (num from-to states, target hold ids)
                string head_line = streamReader.ReadLine();

                MyTools.ParseStringIntoFloatArr(head_line, ref outArr, ','); c_index = 0;

                int from_to_state_count = (int)(outArr[c_index++]);

                user_trajectory_points[i].target_ids[0] = (int)(outArr[c_index++]);
                user_trajectory_points[i].target_ids[1] = (int)(outArr[c_index++]);
                user_trajectory_points[i].target_ids[2] = (int)(outArr[c_index++]);
                user_trajectory_points[i].target_ids[3] = (int)(outArr[c_index++]);

                // load current state
                if (user_trajectory_points[i].cStateID < 0)
                {
                    user_trajectory_points[i].cStateID = memoryManager.GetNextFreeSlotIdx();
                }
                MyTools.ParseStringIntoFloatArr(streamReader.ReadLine(), ref outArr, ','); c_index = 0;

                memoryManager.LoadStateFromList(user_trajectory_points[i].cStateID, ref outArr);

                while (user_trajectory_points[i]._fromToStates.Count > from_to_state_count)
                {
                    user_trajectory_points[i]._fromToStates.RemoveAt(user_trajectory_points[i]._fromToStates.Count - 1);
                }

                for (int s = 0; s < from_to_state_count; s++)
                {
                    if (s >= user_trajectory_points[i]._fromToStates.Count)
                    {
                        user_trajectory_points[i]._fromToStates.Add(memoryManager.GetNextFreeSlotIdx());
                    }
                    if (user_trajectory_points[i]._fromToStates[s] < 0)
                    {
                        user_trajectory_points[i]._fromToStates[s] = memoryManager.GetNextFreeSlotIdx();
                    }

                    MyTools.ParseStringIntoFloatArr(streamReader.ReadLine(), ref outArr, ','); c_index = 0;

                    memoryManager.LoadStateFromList(user_trajectory_points[i]._fromToStates[s], ref outArr);

                    total_states_saved++;
                }
            }
            return;
        }

        void UpdateSampleHighLevelPlan(ref SamplingHighLevelPlan _samplePlan)
        {
            _samplePlan.startingNodeID = -1;
            for (int h = 0; h < 4; h++)
            {
                _samplePlan._sampledFromStanceID[h] = init_hold_ids[h];
                _samplePlan._sampledTargetStanceID[h] = target_hold_ids[h];
            }
            _samplePlan._sampledInitialStateSlotIdx = user_trajectory_points[user_trajectory_points.Count - 1].cStateID;
        }

        void AddTrajectoryPoint(LowLevelController.SamplingTrajectoryStr bestSampleTrajectory)
        {
            user_trajectory_points.Add(new UserTrajectoryPoints());
            user_trajectory_points[user_trajectory_points.Count - 1].cStateID = memoryManager.GetNextFreeSlotIdx();

            controlRig.SaveState(user_trajectory_points[user_trajectory_points.Count - 1].cStateID);

            for (int i = 0; i < 4; i++)
            {
                user_trajectory_points[user_trajectory_points.Count - 1].target_ids[i] = target_hold_ids[i];
            }

            if (bestSampleTrajectory != null)
            {
                for (int i = 0; i < bestSampleTrajectory._sampledMaxStep; i++)
                {
                    user_trajectory_points[user_trajectory_points.Count - 1]._fromToStates.Add(memoryManager.GetNextFreeSlotIdx());

                    memoryManager.SaveState(user_trajectory_points[user_trajectory_points.Count - 1]._fromToStates[i], bestSampleTrajectory._fromToStates[i]);
                }
                total_states_saved += user_trajectory_points[user_trajectory_points.Count - 1]._fromToStates.Count;
            }
            return;
        }

        void RemoveLastTrajectoryPoint()
        {
            current_command_type = UserCommandType.UserNone;
            if (isTaskCompleted)
            {
                if (user_trajectory_points.Count > 1)
                {
                    total_states_saved -= user_trajectory_points[user_trajectory_points.Count - 1]._fromToStates.Count;

                    user_trajectory_points.RemoveAt(user_trajectory_points.Count - 1);
                }
                else
                {
                    for (int b = 0; b < 4; b++)
                        user_trajectory_points[0].target_ids[b] = -1;
                }
            }

            // restoring climber's state and target stance
            controlRig.LoadState(user_trajectory_points[user_trajectory_points.Count - 1].cStateID);

            for (int i = 0; i < 4; i++)
            {
                // restore to current rig's attached hold ids
                init_hold_ids[i] = controlRig.GetCurrentHoldBodyID(i);
                // restore to the last saved trajectory point
                target_hold_ids[i] = user_trajectory_points[user_trajectory_points.Count - 1].target_ids[i];
            }

            isTaskCompleted = true;
            return;
        }

        public float PlayAnimation(float cTime, float scroll_value, bool play_in_loop)
        {
            int index_trajectory_points = cStepAnimation[0];
            int index_state = cStepAnimation[1];
            int scroll_index_target = Mathf.CeilToInt(total_states_saved * scroll_value);
            int scroll_index_current = Mathf.CeilToInt(total_states_saved * current_scroll_value);
            if (Mathf.Abs(scroll_index_target - scroll_index_current) > 0)
            {
                int index_counter = 0;
                for (int p = 0; p < user_trajectory_points.Count; p++)
                {
                    if (index_counter + user_trajectory_points[p]._fromToStates.Count < scroll_index_target)
                    {
                        index_counter += user_trajectory_points[p]._fromToStates.Count;
                    }
                    else
                    {
                        index_trajectory_points = p;
                        index_state = scroll_index_target - index_counter;
                        break;
                    }
                }
                current_scroll_value = scroll_value;
                flag_play_loop = false;
                time_diff_happens = cTime;
            }

            if (cTime - time_diff_happens > 2.0f)
                flag_play_loop = true;

            if (flag_play_loop)
            {
                flag_play_loop = play_in_loop;
            }

            if (index_trajectory_points < user_trajectory_points.Count)
            {
                if (index_state < user_trajectory_points[index_trajectory_points]._fromToStates.Count)
                {
                    controlRig.LoadState(user_trajectory_points[index_trajectory_points]._fromToStates[index_state]);
                }
                else
                {
                    if (flag_play_loop)
                    {
                        index_state = 0;
                        index_trajectory_points++;

                        if (index_trajectory_points >= user_trajectory_points.Count)
                        {
                            index_trajectory_points = 0;
                            scroll_index_target = 0;
                        }
                    }
                }
                if (flag_play_loop)
                {
                    index_state++;
                    scroll_index_target++;
                }
            }

            cStepAnimation[0] = index_trajectory_points;
            cStepAnimation[1] = index_state;
            current_scroll_value = (scroll_index_target / (total_states_saved + 1e-6f));
            return current_scroll_value;
        }
    }

}
