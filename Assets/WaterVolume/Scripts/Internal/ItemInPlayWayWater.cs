#define XWATERVOLUME_USE_PLAYWAY

using UnityEngine;
using System.Collections;
using System.Collections.Generic;
#if WATERVOLUME_USE_PLAYWAY
namespace WaterVolume
{

    public class ItemInPlayWayWater : ItemInWater
    {

        private float precision = 1f;

        private float updateClock = 0;

        Dictionary<Vector3, Vector3> storedVertexNeighbors = new Dictionary<Vector3, Vector3>();


        protected PlayWay.Water.WaterSample[] samples;
        protected Vector3[] sampleDisplacementCache;

        public ItemInPlayWayWater(GameObject go, WaterVolume water, ItemOptions.Options options)
            : base(go, options)
        {
            Init(go, options);
            this.water = water;
            if (water.pwWater != null)
            {

                samples = new PlayWay.Water.WaterSample[bounds.Length + 1];
                sampleDisplacementCache = new Vector3[bounds.Length + 1];
                for (int i = 0; i < bounds.Length; ++i)
                {
                    samples[i] = new PlayWay.Water.WaterSample(water.pwWater, PlayWay.Water.WaterSample.DisplacementMode.HeightAndForces, precision);
                    samples[i].Start(boundsInWorldSpace[i]);
                    sampleDisplacementCache[i] = new Vector3(0, 0, 0);

                }
                samples[bounds.Length] = new PlayWay.Water.WaterSample(water.pwWater, PlayWay.Water.WaterSample.DisplacementMode.HeightAndForces, precision);
                samples[bounds.Length].Start(gameObject.transform.position);
            }
            else
            {
                this.valid = false;
            }


        }



        public override void PrepareForThreadsafeFixedUpdate()
        {
            if (valid)
            {

                base.PrepareForThreadsafeFixedUpdate();
                for (int i = 0; i < bounds.Length; i++)
                {
                    CacheNearestSurfaceVertex(boundsInWorldSpace[i], i);
                }

                CacheNearestSurfaceVertex(position, bounds.Length);
            }

        }

        Vector3 displaced = new Vector3();
        Vector3 flowForce = new Vector3();

        private void CacheNearestSurfaceVertex(Vector3 worldSpaceVector, int index = 0)
        {


#if UNITY_EDITOR
            if (samples != null && samples.Length > index)
#else
                if ((object)samples != null && samples.Length > index)
#endif
            {
              //  if (samples[index].Finished)
               // {
                    samples[index].GetAndResetFaster(worldSpaceVector.x, worldSpaceVector.z, water.playwayWaterTime, ref displaced, ref flowForce);
                    //   displaced = samples[index].GetAndReset(worldSpaceVector.x, worldSpaceVector.z, PlayWay.Water.WaterSample.ComputationsMode.Normal);

                    //sampleDisplacementCache[index] = new Vector3(worldSpaceVector.x, displaced.y, worldSpaceVector.z);
                    sampleDisplacementCache[index].x = worldSpaceVector.x;
                    sampleDisplacementCache[index].y = displaced.y;
                    sampleDisplacementCache[index].z = worldSpaceVector.z;
               // }

            }
        }

        protected override Vector3 NearestSurfaceVertex_TS(Vector3 worldSpaceVector, int index = 0)
        {
            if (!valid)
            {
                return Vector3.zero;
            }
            return sampleDisplacementCache[index];
        }


    }
}
#endif
