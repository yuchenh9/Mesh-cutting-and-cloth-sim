#define XWATERVOLUME_USE_PLAYWAY

using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using UnityEngine;

using UnityEngine.Profiling;
namespace WaterVolume
{
    public class WaterVolume : MonoBehaviour
    {
        private WaterThreads waterThreads;

        public Dictionary<int, ItemInWater> itemsInVolume = new Dictionary<int, ItemInWater>();
        // Cached list copy of dictionary values for faster iteration
        private bool itemsInVolumeListDirty = false;
        private ItemInWater[] itemsInVolumeList;

        // Physical properties
        [Header("Physicality")]
        public float buoyancy = 4.905f;
        public float viscosity = 30;
        [Tooltip("A force vector that will be constantly applied")]
        public Vector3 flow = Vector3.zero;

        [Header("Drafting (drag occlusion)")]
        [Tooltip("Do not apply drag to items trailing close behind others. Adds realism and expense.")]
        public bool simulateDrafting = false;
        [Tooltip("Maximum distance between objects for which a drafting effect can take place.")]
        public float draftingDistance = 1; // meters
        [Tooltip("Speed up. Only applicable if simulating drafting. Only check every other drafting ray per neighbor/item pair unless something connects, then it will check them all. Minimal impact on accuracy.")]
        public bool doSparseDraftingCheck = true;


        [Header("Lift")]
        [Tooltip("Treat everything like a wing shape that produces lift effects. Expensive and unrealistic on non wing shapes. Applying via ItemOptions per wing shaped item is better.")]
        public bool simulateLiftOnAll = false;
        [Tooltip("The amount of lift is varied depending on the angle between the direction of motion and the surface impacted. (The angle of attack.)")]
        public AnimationCurve liftCoefficent;

        [Header("Water Collider")]
        [Tooltip("Specify a BoxCollider set as a trigger to define the shape of the water.")]
        public BoxCollider waterBox;
        [Tooltip("Raise and lower the effective surface of the water.  Has no effect if higher than the top of the Water Box collider.")]
        public float waterLevelOffset = 0;
        [HideInInspector]
        public float waterSurfaceHeight = Mathf.NegativeInfinity;

        [Tooltip("Check for size changes on items already in the volume. Minor performance penalty.")]
        public bool checkForSizeChanges = false;

        [Header("Water Meshes (Optional)")]
        [Tooltip("Enable tracking animated meshes to use as a water surface. The meshes must be inside the \"Water Collider\".")]
        public bool useMeshesAsSurface = false;
        [Tooltip("Breaking a large surface into smaller meshes lets us track them more efficiently.")]
        public GameObject[] waterMeshes;
        [Tooltip("Reading the vertex positions is costly.  Less frequent reads can improve performance with minimal quality degradation. If you have high buoyancy and a quickly animated mesh, you may need a low value to prevent sudden upward movements.")]
        public float meshCacheUpdatePeriod = 0.2f;
        public bool usingPlayWayWater = false;
        public float surfaceReadPeriod = 0.1f;


        [Header("Observer (Optional)")]
        [Tooltip("A GameObject, likely a player or camera, that the water should always behave near.  This lets us avoid making calculations that won't be seen by the observer.")]
        public GameObject observer;
        [HideInInspector]
        public Vector3 observerPosition; // for threaded access
        [Tooltip("Items in water this far from the observer will be made inactive")]
        public float observerDistance = 100;


        [Serializable]
        public class Statistics
        {
            public int itemCount = 0;
            public float fps = 0;
            [HideInInspector]
            public float fpsClock = 0;

        }

        [Header("Threading")]
        [Tooltip("If true, will decide on Start if threads should be used and how many.")]
        public bool automaticThreads = true;
        [Tooltip("Can be toggled at runtime")]
        public bool useThreads = false;
        [Tooltip("Can be changed at runtime")]
        public int threadCount = 2;

        [Header("Misc (Mouseover for details)")]
        private bool notifyEnteringItems = true;
        // Accuracy / efficiency

        [Tooltip("Raycasts greatly affect performance.  Lower spacing means more raycasts. Set spacing to be just smaller than the smallest objects you wish to track")]
        public float raycastSpacing = 0.75f;
        [HideInInspector]
        public float minimumRaycastSpacing = 0.1f;

        [Tooltip("GameObjects that should be tracked but not affected by water. Items can be added here before runtime or during runtime via the OnlyTrack(GameObject) method.")]
        public GameObject[] onlyTrackList;
        private List<int> onlyTrackIDs = new List<int>(); // processed from the ignorelist for efficiency

        [Tooltip("The Water Box Collider must be on a different layer than the items it is tracking for object occlusion to work properly near the surface.  By default it will be forced to the IgnoreRaycast layer, but you can change that here.")]
        public bool dontChangeLayer = false;

        [Tooltip("By default rigidbody drag is set to zero on entry and restored on exit.")]
        public bool dontAlterDrag = false;

        [SerializeField]
        public Statistics statistics = new Statistics();

        // Events
        public delegate void ItemDelegate(ItemInWater item);
        public event ItemDelegate OnItemEnteredWater;
        public event ItemDelegate OnItemExitedWater;
        public event ItemDelegate OnItemCenterEnteredWater;
        public event ItemDelegate OnItemCenterExitedWater;


        #region threading
        int bucketSize;
        int itemCountForThread;
        ManualResetEvent[] threadCompleteEvents;
        #endregion

        [HideInInspector]
        public MeshCache meshCache;

        //Degradations for teaching/debugging purposes
        [HideInInspector]
        public bool useObjectCenterAsUnderWaterCenter = false;
        // [HideInInspector]
        public bool ignoreDistanceFromSurface = false;
        // [HideInInspector]        // public bool applyViscosityOnlyAtCenter = false;

#if WATERVOLUME_USE_PLAYWAY
        public float playwayWaterTime = 0;
        public PlayWay.Water.Water pwWater;

#endif

        // Caching to prevent hit when called from every item
        public float deltaTime = 0;
        public float fixedDeltaTime = 0;

        void Awake()
        {
            CheckInitialConditions();
        }

        private void CheckInitialConditions()
        {

            if (waterBox == null)
            {
                if (gameObject.GetComponent<BoxCollider>() != null)
                {
                    waterBox = gameObject.GetComponent<BoxCollider>();
                }
                else
                {
                    Debug.LogError("You must specify a BoxCollider for the waterBox");
                }
            }

            if (!waterBox.isTrigger)
            {
                Debug.LogWarning("Setting waterBox to be a trigger. Set \"" + waterBox.name + "\"'s collider as trigger manually to avoid this warning.");
                waterBox.isTrigger = true;
            }

            if (!dontChangeLayer)
            {
                waterBox.gameObject.layer = LayerMask.NameToLayer("Ignore Raycast");
            }
            if (useMeshesAsSurface && waterMeshes.Length == 0)
            {
                Debug.LogError("Cannot track water mesh surfaces: No water meshes specified");
                if (waterBox == null)
                {
                    Debug.LogError("Cannot fall back to track water box collider: None specified");
                }
                else
                {
                    Debug.LogWarning("Falling back on tracking water box collider: " + waterBox.name);
                }
            }

            meshCache = new MeshCache();
            meshCache.waterVolume = this;
            meshCache.SetMeshes(waterMeshes);

            if (onlyTrackList.Length > 0)
            {
                // onlyTrackIDs = new List<int>();
                foreach (GameObject go in onlyTrackList)
                {
                    if (go != null)
                    {
                        onlyTrackIDs.Add(go.GetInstanceID());
                    }
                }
            }

            itemsEnumerator = itemsInVolume.GetEnumerator();
            if (automaticThreads)
            {
                threadCount = SystemInfo.processorCount;
                if (threadCount > 1)
                {
                    useThreads = true;
                }
                else
                {
                    useThreads = false;
                }
            }
            waterThreads = new WaterThreads();
            waterThreads.water = this;

            if (Physics.autoSyncTransforms)
            {
                Debug.LogWarning("Auto sync transforms should probably be off");
            }
        }

        public void AddMesh(GameObject obj)
        {
            if (obj == null)
            {
                Debug.LogError("Tried to add null mesh to WaterVolume");
            }
            else
            {
                meshCache.AddMesh(obj);

            }
        }

        public ItemInWater GetItem(int id)
        {
            if (itemsInVolume.ContainsKey(id))
            {
                return itemsInVolume[id];
            }
            else
            {
                //Debug.LogWarning("Item not in volume: " + id);
            }
            return null;

        }

        private ItemInWater probeItem;

        public float GetSurfaceHeight(Vector3 position)
        {
            if (probeItem == null)
            {
                GameObject probe = new GameObject("WaterVolume Probe");
                probe.AddComponent<Rigidbody>().isKinematic = true;
                probe.AddComponent<BoxCollider>().isTrigger = true;
                probeItem = CreateItem(probe, new ItemOptions.Options(), this);
                probeItem.trackOnly = true;

            }
            probeItem.gameObject.transform.position = position;
            probeItem.PrepareForThreadsafeFixedUpdate();
            probeItem.FixedUpdate_TS();
            return position.y + probeItem.distanceToSurface;

        }

        private ItemInWater CreateItem(GameObject go, ItemOptions.Options options, WaterVolume water)
        {
            ItemInWater item;
#if WATERVOLUME_USE_PLAYWAY
            if (usingPlayWayWater)
            {
                if (pwWater == null)
                {
                    pwWater = GameObject.FindObjectOfType<PlayWay.Water.Water>();
                }
                item = new ItemInPlayWayWater(go, this, options);
            }
            else
            {
                item = new ItemInWater(go, options);
            }
#else
            item = new ItemInWater(go, options, this);
#endif
            return item;
        }

        void OnTriggerEnter(Collider other)
        {
            // if(other.name == "Physical") { 
            //  Debug.Log(other.name);
            //  }
            bool trackOnly = onlyTrackIDs.Contains(other.gameObject.GetInstanceID());
            Rigidbody r = other.GetComponent<Rigidbody>();
            ItemOptions io = other.GetComponent<ItemOptions>();
            ItemOptions.Options options = new ItemOptions.Options();
            if (io is ItemOptions)
            {
                options = io.options;
                trackOnly = trackOnly || io.options.trackOnly;
            }
            if ((r != null) || trackOnly)
            {
                if (!r.gameObject.isStatic && !itemsInVolume.ContainsKey(r.gameObject.GetInstanceID()))
                {
                    ItemInWater item = CreateItem(other.gameObject, options, this);

                    if (item.valid)
                    {
                        item.id = other.gameObject.GetInstanceID();
                        item.trackOnly = trackOnly;
                        itemsInVolume.Add(item.id, item);
                        itemsInVolumeListDirty = true;
                        itemsEnumerator = itemsInVolume.GetEnumerator();
                    }
                }
            }
        }

        void OnTriggerExit(Collider other)
        {
            Rigidbody r = other.GetComponent<Rigidbody>();
            if (r != null)
            {

                item = GetItem(r.gameObject.GetInstanceID());
                if (item != null)
                {
                    if (!item.trackOnly)
                    {
                        item.ResetRigidBodyChanges();
                    }
                    itemsInVolume.Remove(r.gameObject.GetInstanceID());
                    itemsInVolumeListDirty = true;
                    itemsEnumerator = itemsInVolume.GetEnumerator();

                    if (!useMeshesAsSurface)
                    {
                        if (OnItemExitedWater != null)
                        {
                            OnItemExitedWater(item);
                        }


                    }
                    NotifyOfExit(item.gameObject);
                    item.valid = false; // It's not going to be reused by water volume. Should get gargabe collected if nothing external is holding a reference.
                }
            }
        }

        private void NotifyOfEntry(ItemInWater item)
        {
            if (notifyEnteringItems)
            {
                item.gameObject.BroadcastMessage("OnEnterWater", item, SendMessageOptions.DontRequireReceiver);
            }
        }

        private void NotifyOfExit(GameObject go)
        {
            if (notifyEnteringItems)
            {
                go.BroadcastMessage("OnExitWater", SendMessageOptions.DontRequireReceiver);
            }
        }


        public void OnlyTrack(GameObject go)
        {
            // Debug.Log("Only track " + go + " " + go.GetInstanceID());
            onlyTrackIDs.Add(go.GetInstanceID());
        }

        float distanceToSurfaceTemp;

        void FixedUpdate()
        {
            fixedDeltaTime = Time.fixedDeltaTime;

#if WATERVOLUME_USE_PLAYWAY
            if (usingPlayWayWater && pwWater != null)
            {
                playwayWaterTime = pwWater.Time;
            }
#endif

            if (waterBox != null)
            {
                waterSurfaceHeight = waterBox.bounds.max.y;
            }

            if (!useMeshesAsSurface)
            {
                if (waterBox == null)
                {
                    Debug.Log("Not tracking water mesh surfaces and no waterbox available");
                }
            }
            else
            {
                if (itemsInVolume.Count > 0)
                {
                    meshCache.FixedUpdate();
                }
            }

            if (observer != null)
            {
                observerPosition = observer.transform.position;
            }

            if (buoyancy != 0 || viscosity != 0)
            {

                if (itemsInVolume.Count > 0)
                {
                    itemsEnumerator = itemsInVolume.GetEnumerator();

                    if (useThreads && threadCount >= 1)
                    {
                        waterThreads.DoFixedUpdate();
                    }
                    else
                    {
                        while (itemsEnumerator.MoveNext())
                        {
                            itemsEnumerator.Current.Value.PrepareForThreadsafeFixedUpdate();
                            itemsEnumerator.Current.Value.FixedUpdate_TS();
                        }
                    }

                    // Clean max 1 invalid item per loop. 
                    itemsEnumerator = itemsInVolume.GetEnumerator();

                    while (itemsEnumerator.MoveNext())
                    {
                        if (!itemsEnumerator.Current.Value.valid)
                        {
                            itemsInVolume.Remove(itemsEnumerator.Current.Key);
                            itemsEnumerator = itemsInVolume.GetEnumerator();
                            break;
                        }
                    }

                }
                if (!useThreads || threadCount <= 1)
                {
                    ForceLoop();
                }


            }

            statistics.itemCount = itemsInVolume.Count;
        }

        void Update()
        {
            deltaTime = Time.deltaTime;
            statistics.fpsClock += Time.deltaTime;
            if(statistics.fpsClock >= 0.3f){
                statistics.fps =(int)(1f / Time.unscaledDeltaTime);
                statistics.fpsClock = 0;
            }
        }

        Vector3 velocityAtPoint = Vector3.zero;

        private ItemInWater item;

        float dragForceMultiplier;

        Dictionary<int, ItemInWater>.Enumerator itemsEnumerator;
        //Ray surfaceFindRay = new Ray();
        //Ray occlusionFindRay = new Ray();


        public virtual void ForceLoop()
        {         

            // Keep the total amount of force constant regardless of variability in time and raycast spacing
            dragForceMultiplier = fixedDeltaTime * raycastSpacing * raycastSpacing * viscosity;

            itemsEnumerator = itemsInVolume.GetEnumerator();

            if (raycastSpacing < minimumRaycastSpacing)
            {
                Debug.LogError("Volume raycast spacing cannot be less than: " + minimumRaycastSpacing);
                raycastSpacing = 1;// minimumRaycastSpacing;
            }

            while (itemsEnumerator.MoveNext()) // using a cached enumerator saves 48B allocation
            {

                item = itemsEnumerator.Current.Value;

                if (!item.valid)
                {
                    continue;
                }
                #region Events

                if (item.eventFlagFirstTouch)
                {
                    if (OnItemEnteredWater != null)
                    {
                        OnItemEnteredWater(item);
                    }
                    NotifyOfEntry(item);

                }

                if (item.eventFlagFullExit)
                {
                    if (OnItemExitedWater != null)
                    {
                        OnItemExitedWater(item);
                    }
                    NotifyOfExit(item.gameObject);
                }


                if (item.eventFlagCenterTouch)
                {
                    if (OnItemCenterEnteredWater != null)
                    {
                        OnItemCenterEnteredWater(item);
                    }
                }


                if (item.eventFlagCenterExit)
                {
                    if (OnItemCenterExitedWater != null)
                    {
                        OnItemCenterExitedWater(item);
                    }
                }
                #endregion
                item.lastDragForceTotal = Vector3.zero;
                item.lastLiftForceTotal = Vector3.zero;

                // Dead simple buoyancy for when the item is outside the range of the observer
                if (!item.isNearObserver && !item.trackOnly) // Just apply viscosity at center
                {
                    if (item.IsInside(item.underWaterCenter))
                    {
                        item.ApplyDragAtPosition(item.backwardVelocity * dragForceMultiplier, item.position, 0, Vector3.zero);
                    }
                }

                else if (!item.trackOnly)
                {

                    for (int i = 0; i < item.dragRayCount; i++)
                    {
                        if (item.dragRayDidHit[i])
                        {
                            Vector3 worldHit = item.dragRayHitPoints[i];
                            Vector3 normal = new Vector3(0, 0, 0);
                            if (item.simulateLift)
                            {
                                normal = item.worldNormals[item.worldNormalIndexPerViscosityRayHitPoint[i]];
                            }

                            // Apply drag, unless we are simulating drafting and this particular ray is blocked
                            if (!simulateDrafting || !item.dragRayBlocked[i])
                            {
                                velocityAtPoint = item.rigidbody.GetPointVelocity(worldHit);
                                velocityAtPoint.x = velocityAtPoint.x * -dragForceMultiplier;
                                velocityAtPoint.y = velocityAtPoint.y * -dragForceMultiplier;
                                velocityAtPoint.z = velocityAtPoint.z * -dragForceMultiplier;
                                item.ApplyDragAtPosition(velocityAtPoint, worldHit, i, normal);
                            }
                        }
                    }

                }

                if (item.underWaterPointCount > 0)
                {
                    if (buoyancy > 0)
                    {
                        item.ApplyBuoyancy();
                    }
                    if (flow != Vector3.zero)
                    {
                        item.rigidbody.AddForceAtPosition(flow, item.underWaterCenter);
                    }
                }


            }
        }

        private bool DoesLineIntersectBox(Vector3 boundingSmallest, Vector3 boundingLargest, Vector3 lineStart, Vector3 lineEnd)
        {
            // Vector3 boundingSmallest = bounds[0];
            // Vector3 boundingLargest = bounds[4];
            if (lineEnd.x < boundingSmallest.x && lineStart.x < boundingSmallest.x) return false;
            if (lineEnd.x > boundingLargest.x && lineStart.x > boundingLargest.x) return false;
            if (lineEnd.y < boundingSmallest.y && lineStart.y < boundingSmallest.y) return false;
            if (lineEnd.y > boundingLargest.y && lineStart.y > boundingLargest.y) return false;
            if (lineEnd.z < boundingSmallest.z && lineStart.z < boundingSmallest.z) return false;
            if (lineEnd.z > boundingLargest.z && lineStart.z > boundingLargest.z) return false;
            return true;
        }

        public ItemInWater[] GetItemsList()
        {
            if (itemsInVolumeListDirty)
            {
                itemsInVolumeList = itemsInVolume.Values.ToArray();
            }
            itemsInVolumeListDirty = false;
            return itemsInVolumeList;
        }
        void OnDisable()
        {
            waterThreads.OnDisable();
        }

    }


}