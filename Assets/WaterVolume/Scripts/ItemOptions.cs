using UnityEngine;
using System.Collections;

// Can be placed on an item to optionally override the global lift setting
namespace WaterVolume
{

    public class ItemOptions : MonoBehaviour
    {
     
        [System.Serializable]
        public class Options
        {
            public bool producesLift = false;
            public bool trackOnly = false;
        }

        public Options options = new Options();

    }

}
