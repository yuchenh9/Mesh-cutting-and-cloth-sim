using UnityEngine;
using UnityEngine.Profiling;

namespace WaterVolume
{

    // Used to prep and cache info about items in water
    public class ItemInWater
    {
        public int id;
        public bool valid = true;
        public bool trackOnly = false;

        #region CachedComponentProperties
        //Some properties are expensive to access or not threadsafe or both
        public GameObject gameObject = null;
        public Transform transform;
        public Rigidbody rigidbody;
        public Collider collider;
        public Matrix4x4 localToWorldMatrix;
        public Matrix4x4 worldToLocalMatrix;
        public Vector3 position;
        public Vector3 localScale;
        public Quaternion rotation;
        #endregion

        private Vector3 lastScale;

        public Vector3[] bounds = new Vector3[8];
        public Bounds scaledLocalBounds;

        // Top down view of vertex layout
        // bottom layer   top layer
        // 6 7             5 4
        // 0 1             3 2
        public Vector3[] boundsInWorldSpace = new Vector3[8];
        public Vector3[] nearestSurfaceVertexPerBound = new Vector3[8];
        public float[] distancesToSurface = new float[8]; // distances for each corner of the bounding box
        public float distanceToSurface = Mathf.NegativeInfinity; // distances for the center
        public float lastDistanceToSurface = Mathf.NegativeInfinity;
        public Vector3 lastDragForceTotal;
        public Vector3 underWaterCenter = Vector3.zero;
        public Vector3 underWaterCenterLast = Vector3.zero;
        public Vector3 underWaterCenterTarget = Vector3.zero;
        public WaterVolume water;



        public float buoyancyForceMultiplier;

        public Vector3 waterEventPoint;



        public float volume = 1;



        #region Drag
        public bool movingEnoughForDrag = true;
        public float scanningWidth;
        public Vector3 pointInFrontOfItem = Vector3.zero;
        public Vector3 forwardVelocity;
        public Vector3 backwardVelocity; //faster to calculate in a sub thread than to derive on main thread
        public Vector3 forwardVelocityNormalized;
        public Vector3 backwardVelocityNormalized;
        public Vector3 perpendicularHorizontal;
        public Vector3 perpendicularVertical;

        private float lastRaycastSpacing = 0;
        public Vector3[] viscosityRayStartPoints;

        // The heart of this whole party is a grid of rays cast from in front of the item in the direction of its movement
        // back towards the item. These are "drag rays".
        public int dragRayCount = 0;

        // Where on the item in world space did the drag ray hit?
        public Vector3[] dragRayHitPoints = new Vector3[0];

        public float[] dragRayHitDistances = new float[0];

        // The grid usually covers more area than the surface of the item,
        // so we track which rays "hit" the item and which do not.
        public bool[] dragRayDidHit = new bool[0];

        // If drafting is turned on, we will check to see if other items occlude the drag rays for this item
        // and store the results in dragRaysBlocked
        public bool[] dragRayBlocked = new bool[0];
        public Vector3[] viscosityRayHitLiftForces = new Vector3[0];

        private float entryDrag;
        private float entryAngularDrag;

        public Ray dragRay;

        #endregion

        #region Drafting
        private Ray draftingRay = new Ray();

        public float diagonalBoundsLength = 0;


        #endregion 

        #region RelevantNeighbors
        public int relevantNeighborCount = 0;
        public ItemInWater[] relevantNeighbors;
        public int overlappingDraftBoxCount = 0;
        public ItemInWater[] overlappingDraftBoxes;
        private float radiusOfEstimatedSphere = 0;

        private int RELEVANT_NEIGHBORS_MAX = 32;
        #endregion

        #region Lift

        public bool simulateLiftRegardlessOfWaterSetting = false;
        public bool simulateLift = false;
        private bool simulatedLiftLastUpdate = false;

        public Vector3 lastLiftForceTotal;

        public Vector3[] localNormals = new Vector3[6]; //The bounds are always a rectangle, so the local normals are always just up,down,back,front,left,right
        public Vector3[] worldNormals = new Vector3[6];
        public int[] worldNormalIndexPerViscosityRayHitPoint = new int[0];
        public float angleOfAttack;

        #endregion

        public ItemInWater(GameObject go, ItemOptions.Options options, WaterVolume water)
        {
            this.water = water;
            Init(go, options);
        }

        protected virtual void Init(GameObject go, ItemOptions.Options options)
        {
            gameObject = go;
            transform = go.transform;
            position = transform.position;
            localScale = transform.localScale;
            rotation = transform.rotation;
            rigidbody = go.GetComponent<Rigidbody>();
            collider = go.GetComponent<Collider>();

            localNormals[0] = new Vector3(1, 0, 0); // Right
            localNormals[1] = new Vector3(-1, 0, 0); // Left
            localNormals[2] = new Vector3(0, 1, 0); // Up
            localNormals[3] = new Vector3(0, -1, 0); // Down
            localNormals[4] = new Vector3(0, 0, 1); // Forward
            localNormals[5] = new Vector3(0, 0, -1); // Back


            entryDrag = rigidbody.drag;
            entryAngularDrag = rigidbody.angularDrag;

            CalculateBounds(go);

            for (int i = 0; i < distancesToSurface.Length; i++)
            {
                distancesToSurface[i] = Mathf.NegativeInfinity;
            }

            for (int i = 0; i < underwaterPoints.Length; i++)
            {
                underwaterPoints[i] = Vector3.zero;
            }

            simulateLiftRegardlessOfWaterSetting = options.producesLift;

            relevantNeighbors = new ItemInWater[RELEVANT_NEIGHBORS_MAX];
            overlappingDraftBoxes = new ItemInWater[RELEVANT_NEIGHBORS_MAX];
        }


        private void CalculateBounds(GameObject go)
        {
            Vector3 boundsCenter = Vector3.zero;
            MeshFilter meshfilter = go.GetComponent<MeshFilter>();
            lastScale = go.transform.lossyScale;

            if (go.GetComponent<BoxCollider>() != null)
            {
                BoxCollider b = go.GetComponent<BoxCollider>();

                bounds[0] = -b.size / 2f;
                bounds[4] = b.size / 2f;

                boundsCenter = b.center;

            }
            else if (meshfilter != null)
            {
                bounds[0] = meshfilter.sharedMesh.bounds.min;
                bounds[4] = meshfilter.sharedMesh.bounds.max;
            }
            else if (go.GetComponent<CapsuleCollider>() != null)
            {
                CapsuleCollider b = go.GetComponent<CapsuleCollider>();
                switch (b.direction)
                {
                    case 0:
                        bounds[0] = new Vector3(-b.height / 2f, -b.radius, -b.radius);
                        bounds[4] = new Vector3(b.height / 2f, b.radius, b.radius);

                        break;
                    case 1:
                        bounds[0] = new Vector3(-b.radius, -b.height / 2f, -b.radius);
                        bounds[4] = new Vector3(b.radius, b.height / 2f, b.radius);

                        break;
                    case 2:
                        bounds[0] = new Vector3(-b.radius, -b.radius, -b.height / 2f);
                        bounds[4] = new Vector3(b.radius, b.radius, b.height / 2f);

                        break;
                }

            }
            else if (go.GetComponent<SphereCollider>() != null)
            {
                SphereCollider b = go.GetComponent<SphereCollider>();
                bounds[0] = new Vector3(-b.radius, -b.radius, -b.radius);
                bounds[4] = new Vector3(b.radius, b.radius, b.radius);
            }
            else
            {

                valid = false;
            }

            if (valid)
            {

                bounds[1].x = -bounds[0].x;
                bounds[1].y = bounds[0].y;
                bounds[1].z = bounds[0].z;

                bounds[1] += boundsCenter;

                bounds[2].x = -bounds[0].x;
                bounds[2].y = -bounds[0].y;
                bounds[2].z = bounds[0].z;

                bounds[2] += boundsCenter;

                bounds[3].x = bounds[0].x;
                bounds[3].y = -bounds[0].y;
                bounds[3].z = bounds[0].z;

                bounds[3] += boundsCenter;


                bounds[5].x = -bounds[4].x;
                bounds[5].y = bounds[4].y;
                bounds[5].z = bounds[4].z;

                bounds[5] += boundsCenter;


                bounds[6].x = -bounds[4].x;
                bounds[6].y = -bounds[4].y;
                bounds[6].z = bounds[4].z;

                bounds[6] += boundsCenter;


                bounds[7].x = bounds[4].x;
                bounds[7].y = -bounds[4].y;
                bounds[7].z = bounds[4].z;

                bounds[7] += boundsCenter;

                bounds[0] += boundsCenter;
                bounds[4] += boundsCenter;



                // used for viscosity scanning.
                // Don't know what orientation its in, so scanning an area that's the square of the longest dimension
                // 4 - 0 is the longest dimension because it's two opposing corners.
                float x = (bounds[4].x - bounds[0].x) * transform.localScale.x;
                float y = (bounds[4].y - bounds[0].y) * transform.localScale.y;
                float z = (bounds[4].z - bounds[0].z) * transform.localScale.z;

                scanningWidth = diagonalBoundsLength = Mathf.Sqrt(Mathf.Pow(x, 2) + Mathf.Pow(y, 2) + Mathf.Pow(z, 2));
                volume = x * y * z;
                radiusOfEstimatedSphere = diagonalBoundsLength / 2f;

                Vector3 size = new Vector3(bounds[4].x - bounds[0].x, bounds[4].y - bounds[0].y, bounds[4].z - bounds[0].z);
                scaledLocalBounds = new Bounds(Vector3.zero, Vector3.Scale(size, transform.localScale));
            }

        }

        // Called when an item exit the water volume
        public void ResetRigidBodyChanges()
        {
            rigidbody.angularDrag = entryAngularDrag;
            rigidbody.drag = entryDrag;
        }

        public void ApplyDragAtPosition(Vector3 force, Vector3 point, int pointIndex, Vector3 normal)
        {
            if (!trackOnly)
            {

                if (force.x == 0 && force.y == 0 && force.z == 0)
                {
                    return;
                }

                if (isNearObserver && simulateLift)
                {
                    // Lift calculations aren't paralelized because if their dependency on
                    // force and force is dependent on knowing the current velocity at the point of interaction.
                    // That information comes from rigidbody.GetPointVelocity which is not threadsafe.

                    Profiler.BeginSample("Calculate lift force");


                    // u == component of drag perpendicular to surface of item (parallel to normal)
                    // is normal normalized?
                    float forceDotNormal = (normal.x * force.x + normal.y * force.y + normal.z * force.z); // Vector3.Dot(force, normal)

                    // Man, it would be nice if Unity's built in Vector3 manipulation classes were more efficient.
                    // Vector3 u = forceDotNormal * normal;
                    Vector3 u = normal;
                    MultiplyVectorBy(ref u, forceDotNormal);

                    // component of drag parallel to surface of item
                    // Vector3 w = force - u;
                    Vector3 w = force;
                    SubtractFromVector(ref w, u);


                    //Vector3 adjustedWaterVector = w.normalized * force.magnitude;
                    Vector3 adjustedWaterVector = w.normalized;
                    MultiplyVectorBy(ref adjustedWaterVector, force.magnitude);

                    //Vector3 adjustmentForce = adjustedWaterVector - force;
                    Vector3 adjustmentForce = adjustedWaterVector;
                    SubtractFromVector(ref adjustmentForce, force);

                    float tmp = forceDotNormal / (Mathf.Sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z) * Mathf.Sqrt(force.x * force.x + force.y * force.y + force.z * force.z));
                    tmp = Mathf.Clamp(tmp, -1f, 1f);
                    float radAngle = Mathf.Acos(tmp); // Vector3.Angle(normal, force) in radians
                    angleOfAttack = Mathf.Rad2Deg * radAngle - 90;

                    // Vector3 liftForce = -adjustmentForce * water.liftCoefficent.Evaluate(angleOfAttack);
                    Vector3 liftForce = adjustmentForce;
                    MultiplyVectorBy(ref liftForce, -1);
                    MultiplyVectorBy(ref liftForce, water.liftCoefficent.Evaluate(angleOfAttack));


                    viscosityRayHitLiftForces[pointIndex] = liftForce;
                    Profiler.EndSample();

                    Vector3 combinedForces = liftForce;
                    AddToVector(ref liftForce, force);
                    rigidbody.AddForceAtPosition(combinedForces, point);
                    AddToVector(ref lastLiftForceTotal, liftForce);
                }
                else
                {
                    rigidbody.AddForceAtPosition(force, point);
                }
                AddToVector(ref lastDragForceTotal, force);
            }
        }


        public void ApplyBuoyancy()
        {
            if (!trackOnly && distanceToSurface > 0)
            {
                Vector3 force = new Vector3(0, 1, 0);
                MultiplyVectorBy(ref force, buoyancyForceMultiplier);
                rigidbody.AddForceAtPosition(force, underWaterCenter);
            }
        }

        public float Depth()
        {
            return distanceToSurface;
        }

        #region Surface tracking
        private GameObject nearestSurfaceChunk;
        public Vector3[] nearestSurfaceChunkVertices;
        int chunkVertexCount = 0;
        int nearestWaterMeshIndex = 0;
        public Vector3[] underwaterPoints = new Vector3[8];

        // Rough approximation of whether the point is inside the water volume.  Is the point you're asking about beneath any of the surface points above?  
        // Won't be accurate for big waves and water edges
        public virtual bool IsInside(Vector3 position)
        {
            for (int i = 0; i < nearestSurfaceVertexPerBound.Length; i++)
            {
                if (position.y < nearestSurfaceVertexPerBound[i].y)
                {
                    return true;
                }
            }
            return false;
        }

        public virtual void PrepareForThreadsafeFixedUpdate()
        {
#if UNITY_EDITOR
            if (!gameObject || !rigidbody || !gameObject.activeInHierarchy)
#else
          //  if ((object)gameObject == null || (object)rigidbody == null || !gameObject.activeInHierarchy) //http://bonusdisc.com/null-references-in-unity/ 
           if(gameObject == null) 
#endif
            {

                valid = false;
                return;
            }

            position = transform.position;
            localScale = transform.localScale;
            rotation = transform.rotation;

            if (water.useMeshesAsSurface)
            {
                nearestWaterMeshIndex = water.meshCache.GetNearestWaterObjectIndex(position);
                nearestSurfaceChunk = water.meshCache.GetWaterObject(nearestWaterMeshIndex);
                nearestSurfaceChunkVertices = water.meshCache.GetMeshVertices(nearestWaterMeshIndex);
                chunkVertexCount = nearestSurfaceChunkVertices.Length;
            }

            buoyancyForceMultiplier = water.fixedDeltaTime * volume * water.buoyancy * 100;

            // for viscosity
            forwardVelocity = rigidbody.velocity;

            simulatedLiftLastUpdate = simulateLift;
            simulateLift = simulateLiftRegardlessOfWaterSetting || water.simulateLiftOnAll;

            worldToLocalMatrix = transform.worldToLocalMatrix; 
            localToWorldMatrix = transform.localToWorldMatrix;
            for (int i = 0; i < bounds.Length; i++)
            {
                boundsInWorldSpace[i] = localToWorldMatrix.MultiplyPoint3x4(bounds[i]);
            }

            if (lastRaycastSpacing != water.raycastSpacing || (water.checkForSizeChanges && lastScale != transform.lossyScale) || (simulateLift && !simulatedLiftLastUpdate))
            {
                if (lastScale != transform.lossyScale)
                {
                    lastScale = transform.lossyScale;
                    CalculateBounds(gameObject);
                }


                lastRaycastSpacing = water.raycastSpacing;
                dragRayCount = 0;

                if (lastRaycastSpacing == 0)
                {
                    Debug.LogError("Raycast spacing cannot be 0");
                    lastRaycastSpacing = 1;
                }

                // Assuming a perfect fit along an axis, the ray count on one axis would be the width / ray spacing plus one
                dragRayCount = (int)Mathf.Pow(Mathf.FloorToInt(scanningWidth / lastRaycastSpacing) + 1, 2);
                // TODO: persist this value and object for a period of time after leaving the water to prevent reallocations when bouncing in and out of the water volume
                viscosityRayStartPoints = new Vector3[dragRayCount];
                dragRayHitPoints = new Vector3[dragRayCount]; // Will be the same as hits, if drafting is off
                dragRayHitDistances = new float[dragRayCount];
                dragRayDidHit = new bool[dragRayCount];
                dragRayBlocked = new bool[dragRayCount];
                viscosityRayHitLiftForces = new Vector3[dragRayCount];

                if (simulateLift)
                {
                    worldNormalIndexPerViscosityRayHitPoint = new int[dragRayCount]; // If there's a hit for every point checked, we need to store a normal (index) for each one
                }

                for (int i = 0; i < dragRayCount; i++)
                {
                    viscosityRayStartPoints[i] = new Vector3(0, 0, 0);
                    dragRayHitPoints[i] = new Vector3(0, 0, 0);
                    dragRayHitDistances[i] = -1;
                    dragRayDidHit[i] = false;
                    dragRayBlocked[i] = false;
                    viscosityRayHitLiftForces[i] = new Vector3(0, 0, 0);
                }



            }

            if (!water.dontAlterDrag)
            {
                rigidbody.angularDrag = 0.25f * underWaterPointCount + entryAngularDrag;

                rigidbody.drag = underWaterPointCount > 0 ? 0 : entryDrag;
            }

            isNearObserver = water.observer == null || (water.observerPosition - position).sqrMagnitude <= water.observerDistance * water.observerDistance;
        }

        protected void ClearUnderwaterPoints()
        {
            
            for (int i = 0; i < underwaterPoints.Length; i++)
            {
                underwaterPoints[i].x = 0;
                underwaterPoints[i].y = Mathf.NegativeInfinity;
                underwaterPoints[i].z = 0;
            }
            underWaterPointCount = 0;
            underWaterCenter.x = underWaterCenter.y = underWaterCenter.z = 0;
            underWaterCenterTarget.x = underWaterCenterTarget.y = underWaterCenterTarget.z = 0;
            
        }

        public int underWaterPointCount = 0;
        public int lastUnderWaterPointCount = 0;

        public bool eventFlagFirstTouch = false;
        public bool eventFlagCenterTouch = false;
        public bool eventFlagCenterExit = false;
        public bool eventFlagFullExit = false;

        public bool isNearObserver = true;
        public virtual void FixedUpdate_TS()
        {
            if (valid)
            {
          
                ClearUnderwaterPoints();
   
   
                for (int i = 0; i < bounds.Length; i++)
                {
                    nearestSurfaceVertexPerBound[i] = NearestSurfaceVertex_TS(boundsInWorldSpace[i], i);

                    distancesToSurface[i] = nearestSurfaceVertexPerBound[i].y - boundsInWorldSpace[i].y;

                    if (distancesToSurface[i] > 0)
                    {
                        underwaterPoints[i] = boundsInWorldSpace[i];
                        underWaterPointCount++;
                        underWaterCenterTarget += underwaterPoints[i];
                    }
                }

                distanceToSurface = NearestSurfaceVertex_TS(position, bounds.Length).y - position.y;

                if (!water.ignoreDistanceFromSurface)
                {
                    buoyancyForceMultiplier *= Mathf.Min(1, distanceToSurface * 10f);
                }


                buoyancyForceMultiplier = Mathf.Max(0, buoyancyForceMultiplier);

                AdjustUnderWaterCenter_TS();


                if (isNearObserver)
                {
                    // for viscocity
                    perpendicularHorizontal = Vector3.Cross(forwardVelocity, Vector3.up);
                    movingEnoughForDrag = false;

                    // Velocity will often be extremely small in one direction but not quite zero
                    if ((forwardVelocity.x > -0.001f || forwardVelocity.x < 0.001f) && (forwardVelocity.z > -0.001f || forwardVelocity.z < 0.001f))
                    {
                        movingEnoughForDrag = true;
                        perpendicularHorizontal = Vector3.Cross(forwardVelocity, Vector3.right); // Really any vector that's not the same direction as forwardVelocity or the inverse will do in the second arg
                    }

                    perpendicularVertical = Vector3.Cross(forwardVelocity, perpendicularHorizontal);

                    forwardVelocityNormalized = forwardVelocity.normalized;
                    backwardVelocityNormalized = -forwardVelocityNormalized;
                    backwardVelocity = -forwardVelocity;

                    perpendicularHorizontal.Normalize();
                    perpendicularVertical.Normalize();



                    if (simulateLift)
                    {
                        #if UNITY_EDITOR
                        if(!water.useThreads){
                            Profiler.BeginSample("Update world normals");
                        }
                        #endif
                        for (int i = 0; i < 6; i++)
                        {
                            worldNormals[i] = (rotation * localNormals[i]).normalized;
                        }
                         #if UNITY_EDITOR
                        if(!water.useThreads){
                            Profiler.EndSample();
                        }
                        #endif
                    }

                    UpdateViscosityRays_TS();
                    
                    if (water.simulateDrafting)
                    {
                        UpdateRelevantNeighbors_TS();
                        UpdateDraftingBlockers_TS(); //TODO: think about updating only 1/2 of the blockers each frame? They won't change so frequently as to need that level of precision.

                    }

                }
                
                UpdateEventFlags_TS();
            }

        }

        protected void UpdateRelevantNeighbors_TS()
        {

            int newCount = 0;
            ItemInWater[] items = water.GetItemsList();
            ItemInWater possibleNeighbor = null;
            for (int i = 0; i < items.Length; i++)
            // while (itemsEnumerator.MoveNext() && newCount < RELEVANT_NEIGHBORS_MAX)
            {
                if (newCount >= RELEVANT_NEIGHBORS_MAX)
                {
                    break;
                }
                possibleNeighbor = items[i];// itemsEnumerator.Current.Value;
                int otherID = possibleNeighbor.id;
                if (otherID != this.id)
                {
                    if (possibleNeighbor.valid)
                    {
                        float maxPossibleInteractionDistanceOfTheseTwoItems = radiusOfEstimatedSphere + possibleNeighbor.radiusOfEstimatedSphere;
                        float maxDistSquared = maxPossibleInteractionDistanceOfTheseTwoItems * maxPossibleInteractionDistanceOfTheseTwoItems;

                        // Instead of calculating the full magnitude of the vector between the center points (which would be comparing spheres),
                        // We're checking a cheaply constructed box that's guaranteed to be larger than the item.
                        // We check one axis a time so we can bail as soon as possible.

                        float xDelta = position.x - possibleNeighbor.position.x;
                        if (xDelta * xDelta < maxDistSquared)
                        {
                            float yDelta = position.y - possibleNeighbor.position.y;
                            if (yDelta * yDelta < maxDistSquared)
                            {
                                float zDelta = position.z - possibleNeighbor.position.z;
                                if (zDelta * zDelta < maxDistSquared)
                                {
                                    relevantNeighbors[newCount] = possibleNeighbor;
                                    newCount++;
                                }
                            }
                        }

                    }
                }
            }
            relevantNeighborCount = newCount;

        }

        protected void UpdateEventFlags_TS()
        {
            eventFlagCenterExit = eventFlagCenterTouch = eventFlagFirstTouch = eventFlagFullExit = false;
            if (lastUnderWaterPointCount == 0)
            {
                if (underWaterPointCount > 0)
                {
                    eventFlagFirstTouch = true;
                    for (int i = 0; i < underwaterPoints.Length; i++)
                    {
                        if (underwaterPoints[i].y > Mathf.NegativeInfinity)
                        {
                            waterEventPoint = underwaterPoints[i];
                            break;
                        }
                    }
                }
            }
            else
            {
                if (underWaterPointCount == 0)
                {
                    eventFlagFullExit = true;
                    waterEventPoint = position;
                    waterEventPoint.y += distanceToSurface;
                }
            }

            if (lastDistanceToSurface < 0 && distanceToSurface > 0)
            {
                eventFlagCenterTouch = true;
                waterEventPoint = position;
            }

            if (lastDistanceToSurface >= 0 && distanceToSurface <= 0)
            {
                eventFlagCenterExit = true;
                waterEventPoint = position;
            }

            lastDistanceToSurface = distanceToSurface;
            lastUnderWaterPointCount = underWaterPointCount;
        }

        protected void AdjustUnderWaterCenter_TS()
        {

            if (underWaterPointCount > 0)
            {
                underWaterCenterTarget = underWaterCenterTarget / underWaterPointCount;

                if (underWaterCenterLast == Vector3.zero)
                {
                    underWaterCenter = underWaterCenterTarget;
                }
                else
                {
                    underWaterCenter = underWaterCenterTarget;
                }
                underWaterCenterLast = underWaterCenter;

                if (water.useObjectCenterAsUnderWaterCenter)
                {
                    underWaterCenter = position;
                    underWaterPointCount = 8;
                }
            }
            buoyancyForceMultiplier *= (float)underWaterPointCount / 8f;
        }

        // negative means above water
        private Vector3 tmpNearestWaterPoint;
        private float tmpShortestDistance;

        Vector3 infiniteUp = new Vector3(0, Mathf.Infinity, 0);
        Vector3 infiniteDown = new Vector3(0, Mathf.NegativeInfinity, 0);
        // Index is useful for playway child class
        protected virtual Vector3 NearestSurfaceVertex_TS(Vector3 worldSpaceVector, int index = 0)
        {
            if (water.useMeshesAsSurface)
            {
                tmpNearestWaterPoint = infiniteUp;
                tmpShortestDistance = Mathf.Infinity;
#if UNITY_EDITOR
                if (!nearestSurfaceChunk)
#else
                if ((object)nearestSurfaceChunk == null)
#endif
                {
                    return infiniteDown;
                }

                for (int i = 0; i < chunkVertexCount; i++)
                {

                    if ((nearestSurfaceChunkVertices[i].x - worldSpaceVector.x) * (nearestSurfaceChunkVertices[i].x - worldSpaceVector.x) +
                        (nearestSurfaceChunkVertices[i].z - worldSpaceVector.z) * (nearestSurfaceChunkVertices[i].z - worldSpaceVector.z) < tmpShortestDistance)
                    {

                        tmpShortestDistance = (nearestSurfaceChunkVertices[i].x - worldSpaceVector.x) * (nearestSurfaceChunkVertices[i].x - worldSpaceVector.x) + (nearestSurfaceChunkVertices[i].z - worldSpaceVector.z) * (nearestSurfaceChunkVertices[i].z - worldSpaceVector.z);
                        tmpNearestWaterPoint = nearestSurfaceChunkVertices[i];
                        tmpNearestWaterPoint.y += water.waterLevelOffset;
                        tmpNearestWaterPoint.y = Mathf.Clamp(tmpNearestWaterPoint.y, tmpNearestWaterPoint.y, water.waterSurfaceHeight);
                    }
                }
                if (tmpShortestDistance != Mathf.Infinity)
                {
                    return tmpNearestWaterPoint;
                }

                return infiniteDown;
            }
            else
            {
                return new Vector3(worldSpaceVector.x, water.waterSurfaceHeight + Mathf.Clamp(water.waterLevelOffset, Mathf.NegativeInfinity, 0), worldSpaceVector.z);

            }

        }

        protected void UpdateViscosityRays_TS()
        {


            if (lastRaycastSpacing == 0)
            {
                //Debug.LogError("Raycast spacing cannot be zero");
                return;
            }

            pointInFrontOfItem = position + ((forwardVelocityNormalized * 1.1f) * scanningWidth / 2f);

            int i = 0;

            // Scan from center out along 4 directions
            for (float x = 0; x <= scanningWidth / 2f; x += lastRaycastSpacing)
            {

                Vector3 pointInFrontOfItemPlusXHorizontal = pointInFrontOfItem;
                // pointInFrontOfItemPlusXHorizontal = (pointInFrontOfItemPlusXHorizontal * x) + pointInFrontOfItemPlusXHorizontal
                Vector3 tmp = perpendicularHorizontal;
                MultiplyVectorBy(ref tmp, x);
                AddToVector(ref pointInFrontOfItemPlusXHorizontal, tmp);

                Vector3 pointInFrontOfItemMinusXHorizontal = pointInFrontOfItem;
                tmp = perpendicularHorizontal;
                MultiplyVectorBy(ref tmp, -x);
                AddToVector(ref pointInFrontOfItemMinusXHorizontal, tmp);

                for (float y = 0; y <= scanningWidth / 2f; y += lastRaycastSpacing)
                {
                    Vector3 perVertY = perpendicularVertical;
                    MultiplyVectorBy(ref perVertY, y);
                    Vector3 perVertNegY = perpendicularVertical;
                    MultiplyVectorBy(ref perVertNegY, -y);

                    viscosityRayStartPoints[i] = pointInFrontOfItemPlusXHorizontal;
                    AddToVector(ref viscosityRayStartPoints[i], perVertY);

                    if (x > 0)
                    {
                        i++;
                        viscosityRayStartPoints[i] = pointInFrontOfItemMinusXHorizontal;
                        AddToVector(ref viscosityRayStartPoints[i], perVertY);
                        if (y > 0)
                        {
                            i++;
                            viscosityRayStartPoints[i] = pointInFrontOfItemMinusXHorizontal;
                            AddToVector(ref viscosityRayStartPoints[i], perVertNegY);

                        }
                    }
                    if (y > 0)
                    {
                        i++;
                        viscosityRayStartPoints[i] = pointInFrontOfItemPlusXHorizontal;
                        AddToVector(ref viscosityRayStartPoints[i], perVertNegY);
                    }
                    i++;
                }
            }

#if UNITY_EDITOR
            if(!water.useThreads){
                // Profiler isn't threadsafe on some older versions of unity
                Profiler.BeginSample("Checking Drag Rays");
            }
#endif

            Vector3 direction = (Quaternion.Inverse(rotation) * backwardVelocity).normalized; // setting it to the ray direction also normalizes it again, but we need it later outside of the ray and it's pricey to access it.
            dragRay.direction = direction;

            Vector3 localHit;
            Vector3 localHitScaled;

            for (i = 0; i < dragRayCount; i++)
            {
                if (movingEnoughForDrag)
                {
                    Vector3 origin = viscosityRayStartPoints[i];
                    float distance;

                    origin = worldToLocalMatrix.MultiplyPoint3x4(origin);

                    if (localScale.x != 1 || localScale.y != 1 || localScale.z != 1)
                    {
                        origin.x = origin.x * localScale.x;
                        origin.y = origin.y * localScale.y;
                        origin.z = origin.z * localScale.z;
                    }

                    dragRay.origin = origin;
                    dragRayDidHit[i] = false;

                    dragRayHitDistances[i] = -1; // -1 is code for didn't hit at all.

                    if (scaledLocalBounds.IntersectRay(dragRay, out distance))
                    {
                        if (distance > 0)
                        {

                            localHit.x = (origin.x + direction.x * distance);
                            localHit.y = (origin.y + direction.y * distance);
                            localHit.z = (origin.z + direction.z * distance);

                            localHitScaled.x = localHit.x / localScale.x;
                            localHitScaled.y = localHit.y / localScale.y;
                            localHitScaled.z = localHit.z / localScale.z;

                            Vector3 worldHit = localToWorldMatrix.MultiplyPoint3x4(localHitScaled);
                            // Is the point of the intersection on the item's boundary underwater? We don't want to apply drag if it's in the air.
                            if (IsInside(worldHit))
                            {
                                dragRayDidHit[i] = true;
                                dragRayHitPoints[i] = worldHit;
                                dragRayHitDistances[i] = distance;
                                dragRayBlocked[i] = false; //Could be. We don't know yet. Resetting here keeps us from having to reset the whole array every update.

                                // Determine the normal relative to the item bounding box for each drag ray that hits
                                if (simulateLift)
                                {
                                    // It's always a box, so there's only 6 normals to choose from
                                    // No need to calculate for each ray. Just figure out which side was hit
                                    // and then mark an index that maps to a list that has the world relative
                                    // normals already calculated.
                                    // Note: When the hit is on an edge exactly, we look at the origin vs localhit to pick the proper side.

                                    // 0 right, 1 left, 2 up, 3 down, 4 front, 5 back.


                                    worldNormalIndexPerViscosityRayHitPoint[i] = 6; // error state

                                    float epsilon = 0.01f;


                                    if (isRoughlyEqual(localHit.x, scaledLocalBounds.extents.x, epsilon) && origin.x > localHit.x)
                                    {
                                        worldNormalIndexPerViscosityRayHitPoint[i] = 0;
                                    }
                                    else if (isRoughlyEqual(localHit.x, -scaledLocalBounds.extents.x, epsilon) && origin.x < localHit.x)
                                    {
                                        worldNormalIndexPerViscosityRayHitPoint[i] = 1;
                                    }
                                    else if (isRoughlyEqual(localHit.y, scaledLocalBounds.extents.y, epsilon) && origin.y > localHit.y)
                                    {
                                        worldNormalIndexPerViscosityRayHitPoint[i] = 2;
                                    }
                                    else if (isRoughlyEqual(localHit.y, -scaledLocalBounds.extents.y, epsilon) && origin.y < localHit.y)
                                    {
                                        worldNormalIndexPerViscosityRayHitPoint[i] = 3;
                                    }
                                    else if (isRoughlyEqual(localHit.z, scaledLocalBounds.extents.z, epsilon) && origin.z > localHit.z)
                                    {
                                        worldNormalIndexPerViscosityRayHitPoint[i] = 4;
                                    }
                                    else if (isRoughlyEqual(localHit.z, -scaledLocalBounds.extents.z, epsilon) && origin.z < localHit.z)
                                    {
                                        worldNormalIndexPerViscosityRayHitPoint[i] = 5;
                                    }

                                    if (worldNormalIndexPerViscosityRayHitPoint[i] == 6)
                                    {
                                        Debug.LogError("LocalHit: " + localHit.x + "," + localHit.y + "," + localHit.z + " scaledLocalBounds.extents: " + scaledLocalBounds.extents.x + "," + scaledLocalBounds.extents.y + "," + scaledLocalBounds.extents.z + " origin: " + origin);
                                        Debug.LogError("Mathf.Approximately( localHit.y, -scaledLocalBounds.extents.y): " + Mathf.Approximately(localHit.y, -scaledLocalBounds.extents.y));
                                    }

                                }
                            }
                        }
                    }

                }


            }
            
            #if UNITY_EDITOR
            if(!water.useThreads){
                // Profiler isn't threadsafe on some older versions of unity
            }
            #endif


        }


        protected void UpdateDraftingBlockers_TS()
        {
            // Raycasting between two boxes will not show a hit if they're perfectly adjacent. So this fudge value just sets the ray back a hair.
            float originOffset = 0.1f;

            // Loop over relevant neighbors
            for (int j = 0; j < relevantNeighborCount; j++)
            {

                ItemInWater potentialBlockingItem = relevantNeighbors[j];

                // We want to cast a ray from the position hit for drag on this item off in the direction of this item's movement.
                // If we hit a neighbor in close proximity with that ray, then we don't want to apply drag.

                // Checking if the occlusion ray hits the neighbor will be done by comparing that ray against the
                // neighbor's scaledLocalBounds. Since those are in the neighbor's local space, we must convert the 
                // item's direction of movement in to the local space of that neighbor.

                Vector3 neighborRelativeDirection = potentialBlockingItem.worldToLocalMatrix.MultiplyVector(-backwardVelocityNormalized);
                draftingRay.direction = neighborRelativeDirection;

                Vector3 worldDirectionPlusOriginOffset = backwardVelocityNormalized * originOffset;


                int increment = 1;
                bool doingSparseDraftCheck = water.doSparseDraftingCheck && dragRayCount > 4;
                if (doingSparseDraftCheck)
                {
                    increment = 2;
                }
                int startOffset = 0;

                bool doAgain = false;
                bool onSecondPass = false;
                do
                {
                    doAgain = false;
                    // Loop over each viscosity ray in the item (not the neighbors)
                    for (int i = startOffset; i < dragRayCount; i = i + increment)
                    {
                        // If any another neighbor is already blocking the ray, we also don't need to look further.
                        if (!dragRayDidHit[i] || dragRayBlocked[i])
                        {
                            continue;
                        }

                        float distance = dragRayHitDistances[i];
                        Vector3 worldRelativeOrigin = dragRayHitPoints[i];

                        // Add fudge value to origin in case of adjacent items
                        worldRelativeOrigin.x += worldDirectionPlusOriginOffset.x;
                        worldRelativeOrigin.y += worldDirectionPlusOriginOffset.y;
                        worldRelativeOrigin.z += worldDirectionPlusOriginOffset.z;


                        // We must also convert the origin of the worldhit in to the local coordinates of the neighbor (plus the fudge value).
                        Vector3 neighborRelativeOrigin = potentialBlockingItem.worldToLocalMatrix.MultiplyPoint3x4(worldRelativeOrigin);

                        // Account for scaling of the neighbor
                        Vector3 neighborLocalScale = potentialBlockingItem.localScale;

                        if (neighborLocalScale.x != 1 || neighborLocalScale.y != 1 || neighborLocalScale.z != 1)
                        {
                            neighborRelativeOrigin.x = neighborRelativeOrigin.x * neighborLocalScale.x;
                            neighborRelativeOrigin.y = neighborRelativeOrigin.y * neighborLocalScale.y;
                            neighborRelativeOrigin.z = neighborRelativeOrigin.z * neighborLocalScale.z;
                        }

                        draftingRay.origin = neighborRelativeOrigin;

                        if (potentialBlockingItem.scaledLocalBounds.IntersectRay(draftingRay, out distance))
                        {

                            //Debug.Log(potentialBlockingItem.scaledLocalBounds.center+" - "+potentialBlockingItem.scaledLocalBounds.extents);
                            Vector3 localDistanceVector = neighborRelativeDirection * distance;
                            Vector3 worldDistanceVector = Vector3.Scale(localDistanceVector, potentialBlockingItem.localScale);

                            float worldDistance = worldDistanceVector.magnitude;


                            if (distance > 0 && worldDistance < water.draftingDistance)
                            {
                                dragRayBlocked[i] = true;
                                if(!onSecondPass && doingSparseDraftCheck){
                                    doAgain = true;
                                    startOffset = 1;
                                }
                            }

                        }


                    }
                    onSecondPass = true;                
                } while (doAgain);


            }
        }

        #endregion

        #region Utilities
        // Much faster that v1 + v2. Yay.
        protected void AddToVector(ref Vector3 v1, Vector3 v2)
        {
            v1.x = v1.x + v2.x;
            v1.y = v1.y + v2.y;
            v1.z = v1.z + v2.z;
        }

        protected void SubtractFromVector(ref Vector3 v1, Vector3 v2)
        {
            v1.x = v1.x - v2.x;
            v1.y = v1.y - v2.y;
            v1.z = v1.z - v2.z;
        }

        protected void MultiplyVectorBy(ref Vector3 v1, float f)
        {
            v1.x = v1.x * f;
            v1.y = v1.y * f;
            v1.z = v1.z * f;
        }


        bool isRoughlyEqual(float a, float b, float epsilon)
        {
            if (a >= b - epsilon && a <= b + epsilon)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        #endregion
    }


}
