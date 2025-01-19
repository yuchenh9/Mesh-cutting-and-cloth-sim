using UnityEngine;
using System.Collections;
/*

    ItemInWater instances are not monobehaviors, so it's not convenient to view their current properties.
    Put this script on a GameObject with a collider and rigidbody, add a reference to the WaterVolume, and then you can use the inspector to view the forces at play.
*/
namespace WaterVolume
{
    public class ItemInspectorExample : MonoBehaviour
    {

        public WaterVolume water;
        private ItemInWater item;

        public int id;

        public bool isNearObserver = false;
        public Vector3[] nearestSurfaceVertexPerBound = new Vector3[8];
        public float distanceToSurface = Mathf.NegativeInfinity; // distances for the center
        public Vector3 lastDragForceTotal;
        public Vector3 underWaterCenter = Vector3.zero;
        public bool simulateLiftRegardlessOfWaterSetting = false;
        public Vector3 lastLiftForceTotal;
        public float angleOfAttack;

        public bool valid = true;
        public bool trackOnly = false;

        public Vector3 forwardVelocity;
        public int viscosityRayCount = 0;

        public float longestDimension;

        public float volume;

        public int lastUnderWaterPointCount = 0;

        public ItemInWater[] overlappingDraftBoxes;
        public int overlappingDraftBoxCount = 0;
        public int relevantNeighborCount = 0;
        public ItemInWater[] relevantNeighbors;

       
        public Vector3 scaledLocalBoundsExtents;

        public void Update()
        {
            if (water != null)
            {

                if (item == null || !item.valid)
                {
                    item = water.GetItem(gameObject.GetInstanceID());
                }


                if (item != null)
                {
                    id = item.id;

                    isNearObserver = item.isNearObserver;
                    longestDimension = item.scanningWidth;

                    valid = item.valid;
                    trackOnly = item.trackOnly;


                    nearestSurfaceVertexPerBound = item.nearestSurfaceVertexPerBound;
                    distanceToSurface = item.distanceToSurface;


                    lastDragForceTotal = item.lastDragForceTotal;
                    underWaterCenter = item.underWaterCenter;

                    simulateLiftRegardlessOfWaterSetting = item.simulateLiftRegardlessOfWaterSetting;
                    lastLiftForceTotal = item.lastLiftForceTotal;
                    angleOfAttack = item.angleOfAttack;

                    forwardVelocity = item.forwardVelocity;
                    viscosityRayCount = item.dragRayCount;

                    volume = item.volume;
                    lastUnderWaterPointCount = item.lastUnderWaterPointCount;

                    overlappingDraftBoxCount = item.overlappingDraftBoxCount;
                    overlappingDraftBoxes = item.overlappingDraftBoxes;

                    relevantNeighborCount = item.relevantNeighborCount;
                    relevantNeighbors = item.relevantNeighbors;
                    scaledLocalBoundsExtents = item.scaledLocalBounds.extents;

                }
            }
        }

    }
}
