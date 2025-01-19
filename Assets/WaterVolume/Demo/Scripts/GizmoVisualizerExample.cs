using UnityEngine;

namespace WaterVolume
{
    public class GizmoVisualizerExample : MonoBehaviour
    {

        public WaterVolume water;

        public bool dragRays = true;
        public bool dragSpheres = true;
        public bool lift = true;
        public bool buoyancy = true;
        public bool bounds = true;
        public bool relationshipToSurface = true;
        public bool extra = false;
        public bool normals = false;

        public bool sphericalApproximation = false;

        private Color[]
            normalColors = new Color[6]
            {
                Color.red, new Color(0.5f,0,0), Color.green, new Color(0,0.5f,0), Color.blue, new Color(0,0,0.5f)
            };

        public void OnDrawGizmos()
        {
            foreach (var kvp in water.itemsInVolume)
            {
                var item = kvp.Value;
                if (kvp.Value.gameObject == null)
                {
                    continue; // An object has been deleted or exited the water during this loop.
                }
                if (relationshipToSurface)
                {
                    // For each of the corners of the item that are under water
                    for (int i = 0; i < kvp.Value.underwaterPoints.Length; i++)
                    {
                        // Mark the underwater corner
                        Gizmos.color = Color.yellow;
                        Gizmos.DrawWireSphere(kvp.Value.underwaterPoints[i], 0.2f);

                        // Draw lines from the underwater corner to the surface
                        Gizmos.color = Color.green;
                        Gizmos.DrawLine(kvp.Value.underwaterPoints[i], kvp.Value.underwaterPoints[i] + Vector3.up * kvp.Value.distancesToSurface[i]);

                        // Circle the nearest surface vertex to the underwater corner
                        Gizmos.DrawWireSphere(kvp.Value.nearestSurfaceVertexPerBound[i], 0.2f);
                    }
                }

                if (buoyancy)
                {
                    // Draw a line to illustrate buoyancy
                    Gizmos.color = Color.blue;
                    Gizmos.DrawLine(kvp.Value.underWaterCenter, kvp.Value.underWaterCenter + Vector3.up * kvp.Value.buoyancyForceMultiplier);
                }

                if (dragRays)
                {
                    for (int j = 0; j < item.dragRayCount; j++)
                    {
                        if (item.dragRayDidHit[j])
                        {
                            Gizmos.color = Color.red;

                            if (item.dragRayBlocked[j])
                            {
                                Gizmos.color = Color.grey;
                            }
                            Gizmos.DrawLine(item.viscosityRayStartPoints[j], item.dragRayHitPoints[j]);
                        }
                    }

                }

                if (dragSpheres)
                {

                    for (int j = 0; j < item.dragRayCount; j++)
                    {
                        if (item.dragRayDidHit[j])
                        {
                            Gizmos.color = Color.red;

                            if (item.dragRayBlocked[j])
                            {
                                Gizmos.color = Color.grey;
                            }
                            Gizmos.DrawSphere(item.dragRayHitPoints[j], 0.03f);

                        }
                    }

                }


                if (normals)
                {
                    for (int j = 0; j < 6; j++)
                    {
                        Gizmos.color = normalColors[j];
                        Gizmos.DrawLine(item.position, item.position + item.worldNormals[(int)j]);
                    }


                }

                if (lift)
                {
                    if (item.simulateLift)
                    {


                        // Draw each viscosity lift vector
                        for (int j = 0; j < item.dragRayCount; j++)
                        {
                            if (item.dragRayDidHit[j] && !item.dragRayBlocked[j])
                            {
                                Gizmos.color = Color.yellow;
                                Gizmos.DrawLine(item.dragRayHitPoints[j], item.dragRayHitPoints[j] + item.viscosityRayHitLiftForces[j]);
                            }
                        }

                        //Normals used in lift calculation
                        /*
                        for (int j = 0; j < item.viscosityRayCount; j++)
                        {
                            if (item.viscosityRayCheckDistances[j] != -1 && !item.viscosityRaysBlocked[j])
                            {
                                int normalIndex = item.worldNormalIndexPerViscosityRayHitPoint[j];

                                if (normalIndex != 6)
                                {
                                    Gizmos.color = normalColors[normalIndex];

                                    Gizmos.DrawLine(item.viscosityRayCheckPoints2[j], item.viscosityRayCheckPoints2[j] + item.worldNormals[normalIndex]);
                                }
                                else
                                {
                                    Gizmos.color = Color.magenta;

                                    Gizmos.DrawSphere(item.viscosityRayCheckPoints2[j], 0.05f);

                                }
                            }
                        }
                        */
                    }
                }


                if (bounds)
                {


                    /*

                    Gizmos.color = Color.yellow;
                    Gizmos.DrawWireSphere(kvp.Value.boundsInWorldSpace[0], 0.2f);

                    Gizmos.color = Color.green;
                    Gizmos.DrawWireSphere(kvp.Value.boundsInWorldSpace[1], 0.2f);

                    Gizmos.color = Color.cyan;
                    Gizmos.DrawWireSphere(kvp.Value.boundsInWorldSpace[2], 0.2f);

                    Gizmos.color = Color.magenta;
                    Gizmos.DrawWireSphere(kvp.Value.boundsInWorldSpace[3], 0.2f);

                    Gizmos.color = Color.red;
                    Gizmos.DrawWireSphere(kvp.Value.boundsInWorldSpace[4], 0.2f);

                      Gizmos.color = Color.white;
                    Gizmos.DrawWireSphere(kvp.Value.boundsInWorldSpace[5], 0.2f);

                    Gizmos.color = Color.black;
                    Gizmos.DrawWireSphere(kvp.Value.boundsInWorldSpace[6], 0.2f);


                    foreach (var b in kvp.Value.bounds)
                    {
                        // Vector3 boundsMinimumLocalScaled = item.transform.InverseTransformPoint(item.boundsInWorldSpace[0]);
                        // Vector3 boundsMaximumLocalScaled = item.transform.InverseTransformPoint(item.boundsInWorldSpace[4]);

                        Gizmos.color = Color.green;
                        Gizmos.DrawWireSphere(Vector3.Scale(b, kvp.Value.transform.localScale), 0.2f);

                    }
                    // Show the viscosity scan raycasts
                    if (kvp.Value.distanceToSurface > 0)
                    {

                        foreach (Vector3 start in kvp.Value.viscosityRayStartPoints)
                        {
                            Gizmos.color = Color.blue;
                            Vector3 direction = kvp.Value.backwardVelocityNormalized;
                            Vector3 origin = start;
                            origin = kvp.Value.transform.InverseTransformPoint(origin);
                            origin = Vector3.Scale(origin, kvp.Value.transform.localScale);
                            direction = kvp.Value.transform.InverseTransformDirection(direction);

                            Vector3 size = new Vector3(kvp.Value.bounds[4].x - kvp.Value.bounds[0].x, kvp.Value.bounds[4].y - kvp.Value.bounds[0].y, kvp.Value.bounds[4].z - kvp.Value.bounds[0].z);
                            Bounds bounds = new Bounds(Vector3.zero, Vector3.Scale(size, kvp.Value.transform.localScale));
                            // Bounds bounds = new Bounds(Vector3.zero, kvp.Value.transform.localScale);
                            Gizmos.DrawWireSphere(bounds.max, 0.1f);
                            Gizmos.DrawWireSphere(bounds.min, 0.1f);

                            Ray ray = new Ray(origin, direction);
                            float distance;
                            if (bounds.IntersectRay(ray, out distance))
                            {
                                Vector3 localHit = origin + direction * distance;
                                Vector3 worldHit = kvp.Value.transform.TransformPoint(new Vector3(localHit.x / kvp.Value.transform.localScale.x, localHit.y / kvp.Value.transform.localScale.y, localHit.z / kvp.Value.transform.localScale.z));
                                // if (DoesLineIntersectBox(boundsMinimumLocalScaled, boundsMaximumLocalScaled, origin, origin + direction * kvp.Value.scanningWidth)){
                                Gizmos.color = Color.green;
                                Gizmos.DrawLine(origin, origin + (direction * kvp.Value.scanningWidth));
                                Gizmos.DrawSphere(localHit, 0.2f);
                                Gizmos.DrawSphere(worldHit, 0.2f);

                            }
                            else
                            {
                                Gizmos.DrawSphere(origin, 0.1f);

                            }



                        }
                    }
                    */

                }

                if (sphericalApproximation)
                {
                    Gizmos.color = Color.green;
                    Gizmos.DrawWireSphere(kvp.Value.transform.position, kvp.Value.diagonalBoundsLength / 2f);
                }

                if (extra)
                {
                    // Draw a cross in the direction of motion
                    Gizmos.color = Color.red;

                    Gizmos.DrawLine(kvp.Value.pointInFrontOfItem + (-kvp.Value.scanningWidth / 2f * kvp.Value.perpendicularHorizontal), kvp.Value.pointInFrontOfItem + (kvp.Value.scanningWidth / 2f * kvp.Value.perpendicularHorizontal));
                    Gizmos.color = Color.blue;
                    Gizmos.DrawLine(kvp.Value.pointInFrontOfItem + (-kvp.Value.scanningWidth / 2f * kvp.Value.perpendicularVertical), kvp.Value.pointInFrontOfItem + (kvp.Value.scanningWidth / 2f * kvp.Value.perpendicularVertical));

                    // Show the viscosity scan raycasts
                    if (item.distanceToSurface > 0)
                    {
                        //TODO: wtf is the space of the viscosityRayStartPoints?
                        foreach (Vector3 start in item.viscosityRayStartPoints)
                        {
                            Gizmos.color = Color.cyan;
                            //Vector3 worldStart = item.localToWorldMatrix.MultiplyVector(start);
                            //worldStart = Quaternion.Inverse(item.rotation) * start;
                            Gizmos.DrawLine(start, start + (item.backwardVelocityNormalized * item.scanningWidth));
                        }

                    }


                }
            }

        }
    }

}