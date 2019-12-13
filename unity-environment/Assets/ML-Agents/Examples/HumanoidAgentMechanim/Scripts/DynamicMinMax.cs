using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;


class DynamicMinMax
{
    class Node
    {
        public Node()
        {

        }

        public int[] children = { -1, -1};
        public int parent;
        public int root;
        public bool hasChildren = false;
        public int elemIdx;
        public int minChildIdx, maxChildIdx;
        public double value;
        public List<int> leaves = new List<int>();
    }

    List<Node> mNodes = new List<Node>();

    static int cIndexOnNodes = 0;

    static int getNextIndex()
    {
        return cIndexOnNodes++;
    }

    void Init(int cIndex, int N, int iParent)
    {
        while (cIndex >= mNodes.Count)
            mNodes.Add(new Node());

        mNodes[cIndex].hasChildren = false;

        mNodes[cIndex].children[0] = -1;
        mNodes[cIndex].children[1] = -1;
        mNodes[cIndex].parent = iParent;
        mNodes[cIndex].root = cIndex;
        mNodes[cIndex].hasChildren = false;
        int counter = 0;
        while (mNodes[mNodes[cIndex].root].parent != -1)
        {
            mNodes[cIndex].root = mNodes[mNodes[cIndex].root].parent;
            counter++;
            if (counter > 1000)
                break;
        }

        if (N > 1)
        {
            mNodes[cIndex].hasChildren = true;
            //divide until we have a child for each discrete pdf element. 
            //also gather the subtree leaves to the leaves vector
            int[] NChildren = { N / 2, N - N / 2 };
            for (int k = 0; k < 2; k++)
            {
                mNodes[cIndex].children[k] = DynamicMinMax.getNextIndex();
                Init(mNodes[cIndex].children[k], NChildren[k], cIndex);

                for (int i = 0; i < mNodes[mNodes[cIndex].children[k]].leaves.Count; i++)
                {
                    mNodes[cIndex].leaves.Add(mNodes[mNodes[cIndex].children[k]].leaves[i]);
                }
                //children[k] = new DynamicMinMax();
                //children[k].Init(NChildren[k], this);
                //for (int i = 0; i < children[k].leaves.Count; i++)
                //{
                //    leaves.Add(children[k].leaves[i]);
                //}
            }

        }
        else
        {
            mNodes[cIndex].leaves.Add(cIndex);
        }

        //at root, update the pdf element (bin) indices of all leaves
        if (mNodes[cIndex].parent == -1)
        {
            for (int i = 0; i < mNodes[cIndex].leaves.Count; i++)
            {
                mNodes[mNodes[cIndex].leaves[i]].elemIdx = i;
                mNodes[mNodes[cIndex].leaves[i]].minChildIdx = i;
                mNodes[mNodes[cIndex].leaves[i]].maxChildIdx = i;
            }
        }

        //init minChildIdx and maxChildIdx to 0 (doesn't matter what, just something that will not point outside the allocated data)
        mNodes[cIndex].minChildIdx = 0;
        mNodes[cIndex].maxChildIdx = 0;
    }


    public DynamicMinMax(int N)
    {
        Init(getNextIndex(), N, -1);
    }

    public void DestroyMinMax()
    {
        cIndexOnNodes = 0;
    }

    public void SetValue(int idx, double value)
    {
        mNodes[mNodes[0].leaves[idx]].value = value;
        int p = mNodes[0].leaves[idx];
        while (mNodes[p].parent != -1)
        {
            p = mNodes[p].parent;
            if (mNodes[p].hasChildren)
            {
                mNodes[p].minChildIdx = mNodes[mNodes[0].leaves[mNodes[mNodes[p].children[0]].minChildIdx]].value < mNodes[mNodes[0].leaves[mNodes[mNodes[p].children[1]].minChildIdx]].value
                    ? mNodes[mNodes[p].children[0]].minChildIdx : mNodes[mNodes[p].children[1]].minChildIdx;
                mNodes[p].maxChildIdx = mNodes[mNodes[0].leaves[mNodes[mNodes[p].children[0]].maxChildIdx]].value < mNodes[mNodes[0].leaves[mNodes[mNodes[p].children[1]].maxChildIdx]].value
                    ? mNodes[mNodes[p].children[0]].maxChildIdx : mNodes[mNodes[p].children[1]].maxChildIdx;
            }
        }
    }

    double getValue(int idx)
    {
        return mNodes[mNodes[0].leaves[idx]].value;
    }

    public int GetMinIdx()
    {
        return mNodes[0].minChildIdx;
    }
    int getMaxIdx()
    {
        return mNodes[0].maxChildIdx;
    }
}

