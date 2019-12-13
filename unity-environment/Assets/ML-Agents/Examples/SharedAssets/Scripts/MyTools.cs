using System.Collections.Generic;
using UnityEngine;

namespace Assets.ML_Agents.Examples.SharedAssets.Scripts
{
    public class MyTools
    {
        public static float GetSquared(float x)
        {
            return x * x;
        }

        public static float GetAngularDistance(Quaternion p, Quaternion q)
        {
            return 2.0f * Mathf.Acos(Mathf.Abs(Quaternion.Dot(p, q)) - 1e-6f);
        }

        public static void ParseStringIntoListFloatArr(ref string msg, ref List<float>[] _outArr, char s = '|')
        {
            char[] separator = { s };
            string[] str_arr = msg.Split(separator);

            int counter_on_str_arr = 0;
            int batch_size = (int)float.Parse(str_arr[counter_on_str_arr]); counter_on_str_arr++;
            int sample_size = (int)(str_arr.Length / batch_size);
            for (int i = 0; i < batch_size && i < _outArr.Length; i++)
            {
                _outArr[i].Clear();
                for (int j = 0; j < sample_size; j++)
                {
                    try
                    {
                        _outArr[i].Add(float.Parse(str_arr[counter_on_str_arr])); counter_on_str_arr++;
                    }
                    catch
                    {

                    }
                }
            }
        }

        public static void RollInArray(ref List<int> arr)
        {
            int temp_val = arr[0];

            arr.RemoveAt(0);
            arr.Add(temp_val);

            return;
        }

        public static void ParseStringIntoFloatArr(string msg, ref List<float> outArr, char s = '|')
        {
            outArr.Clear();

            char[] separator = { s };
            string[] str_arr = msg.Split(separator);

            for (int i = 0; i < str_arr.Length; i++)
            {
                try
                {
                    outArr.Add(float.Parse(str_arr[i]));
                }
                catch
                {

                }
            }

            return;
        }

        public static string ParseListArrFloatInToString(ref List<float[]> arrStateFeature)
        {
            string ret_str = "";

            if (arrStateFeature.Count > 0)
            {
                ret_str = arrStateFeature.Count.ToString() + "|";
                for (int i = 0; i < arrStateFeature.Count - 1; i++)
                {
                    ret_str += ParseArrFloatInToString(arrStateFeature[i]) + "|";
                }
                ret_str += ParseArrFloatInToString(arrStateFeature[arrStateFeature.Count - 1]);
            }
            return ret_str;
        }

        public static string ParseArrFloatInToString(float[] stateFeature)
        {
            string ret_str = "";

            for (int i = 0; i < stateFeature.Length; i++)
            {
                if (i == stateFeature.Length - 1)
                    ret_str += stateFeature[i].ToString();
                else
                    ret_str += stateFeature[i].ToString() + "|";
            }

            return ret_str;
        }

        public static string ParseIntoString(char id, string data)
        {
            string message = "";
            message += id;
            message += '|';
            message += data;
            return message;
        }

        public static bool IsStanceAEqualStanceB(int[] _stanceA, int[] _stanceB)
        {
            if (_stanceA.Length != _stanceB.Length)
                return false;
            for (int b = 0; b < 4; b++)
            {
                if (_stanceA[b] != _stanceB[b])
                    return false;
            }

            return true;
        }

        public static int GetNumConnectedHoldsInStance(int[] _stanceA)
        {
            int _counter = 0;
            for (int b = 0; b < 4; b++)
            {
                if (_stanceA[b] >= 0)
                    _counter++;
            }

            return _counter;
        }

        public static int GetDiffBtwSetASetB(int[] set_a, int[] set_b)
        {
            int mCount = 0;
            for (int i = 0; i < set_a.Length; i++)
            {
                if (set_a[i] != set_b[i])
                {
                    mCount++;
                }
            }
            return mCount;
        }

        public static void Shuffle(ref int[] array)
        {
            // Random random = new Random();
            int n = array.Length;
            while (n > 1)
            {
                n--;
                int i = UnityEngine.Random.Range(0, n + 1);
                int temp = array[i];
                array[i] = array[n];
                array[n] = temp;
            }

            return;
        }

        public static bool IsInSetIDs(int _id, List<int> iSetIDs)
        {
            for (int i = 0; i < iSetIDs.Count; i++)
            {
                if (iSetIDs[i] == _id)
                {
                    return true;
                }
            }
            return false;
        }

        public static bool AddToSetIDs(int _id, List<int> iSetIDs)
        {
            if (!IsInSetIDs(_id, iSetIDs))
            {
                iSetIDs.Add(_id);
                return true;
            }
            return false;
        }

        public static int LoadFromList(int cIndex, ref List<float> inListVal, ref Vector3 outVal)
        {
            outVal[0] = inListVal[cIndex++];
            outVal[1] = inListVal[cIndex++];
            outVal[2] = inListVal[cIndex++];
            return cIndex;
        }

        public static int LoadFromList(int cIndex, ref List<float> inListVal, ref Quaternion outVal)
        {
            outVal[0] = inListVal[cIndex++];
            outVal[1] = inListVal[cIndex++];
            outVal[2] = inListVal[cIndex++];
            outVal[3] = inListVal[cIndex++];
            return cIndex;
        }

        public static int LoadFromList(int cIndex, ref List<float> inListVal, ref float outVal)
        {
            outVal = inListVal[cIndex++];
            return cIndex;
        }

        public static void PushStateFeature(ref List<float> outState, Vector3 _v)
        {
            outState.Add(_v[0]);
            outState.Add(_v[1]);
            outState.Add(_v[2]);
        }

        public static void PushStateFeature(ref List<float> outState, Quaternion _v)
        {
            outState.Add(_v[0]);
            outState.Add(_v[1]);
            outState.Add(_v[2]);
            outState.Add(_v[3]);
        }

        public static void PushStateFeature(ref List<float> outState, float _v)
        {
            outState.Add(_v);
        }

        public static double NextGussian(double mean, double stdDev)
        {
            float u1 = 1.0f - UnityEngine.Random.value; //uniform(0,1] random doubles
            float u2 = 1.0f - UnityEngine.Random.value;
            float randStdNormal = Mathf.Sqrt(-2.0f * Mathf.Log(u1)) *
                         Mathf.Sin(2.0f * Mathf.PI * u2); //random normal(0,1)
            return mean + stdDev * randStdNormal; //random normal(mean,stdDev^2)
        }
    };

}
