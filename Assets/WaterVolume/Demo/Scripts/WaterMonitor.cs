using UnityEngine;
using System.Collections;

namespace WaterVolume
{
    public class WaterMonitor : MonoBehaviour
    {

        public int waterItemID = -1;
        public bool isUnderWater = false;

		public virtual void OnEnterWater(ItemInWater item)
        {
			waterItemID = item.id;
            isUnderWater = true;
        }

        public virtual void OnExitWater()
        {
            waterItemID = -1;
            isUnderWater = false;
        }
    }
}
