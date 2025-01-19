using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using System.Collections.Generic;

namespace WaterVolume
{
    public class WaterVolumeTestUI : MonoBehaviour
    {

        #region testObjects
        private List<GameObject> testObjects;
        public GameObject plankPrefab;
        public GameObject longPlankPrefab;
        public GameObject cubePrefab;
        public GameObject bigCubePrefab;
        public GameObject cylinderPrefab;
        public GameObject pontoonBoatPrefab;
        #endregion

        public GameObject spawnPoint;
        public WaterVolume waterVolume;

        public Vector3 spawnVelocity = Vector3.zero;



        #region stats, sliders and checkboxes
        public Text itemCountText;
        public Text fpsText;
        public Text raySpacingText;
        public Slider spaceBetweenRaycastsSlider;


        public Text viscosityText;
        public Slider viscositySlider;


        public Text buoyancyText;
        public Slider buoyancySlider;


        public Text waveScaleText;
        public Slider waveScaleSlider;

        public Text waveSpeedText;
        public Slider waveSpeedSlider;

        public Text waterFlowText;
        public Slider waterFlowSlider;

        public Toggle threading;
        public Toggle useMeshes;
        public Toggle splashing;
        public Toggle drafting;

        public GameObject animatedWaterMesh;
        public GameObject staticWaterVisual;

        public InputField spawnCount;

        public SplashExample splasher;

        #endregion

        public GameObject threadingErrorMessage;
        public GameObject downloadButton;

        public GameObject exitButton;

        // Use this for initialization
        void Start()
        {
            #if UNITY_WEBGL
                waterVolume.useThreads = false;
                waterVolume.simulateDrafting = false;
                threadingErrorMessage.SetActive(true);
                downloadButton.SetActive(true);
                threading.gameObject.SetActive(false);
                exitButton.SetActive(false);
            #endif


            testObjects = new List<GameObject>();

            raySpacingText.text = waterVolume.raycastSpacing.ToString();
            spaceBetweenRaycastsSlider.value = waterVolume.raycastSpacing;

            viscosityText.text = waterVolume.viscosity.ToString();
            viscositySlider.value = waterVolume.viscosity;

            buoyancyText.text = waterVolume.buoyancy.ToString();
            buoyancySlider.value = waterVolume.buoyancy;


            waveScaleText.text = waveScaleSlider.value.ToString();
            waveSpeedText.text = waveSpeedSlider.value.ToString();

            animatedWaterMesh.gameObject.SetActive(waterVolume.useMeshesAsSurface);
            staticWaterVisual.gameObject.SetActive(!waterVolume.useMeshesAsSurface);

            splashing.isOn = splasher.gameObject.activeInHierarchy;
            drafting.isOn = waterVolume.simulateDrafting;

            foreach (GameObject go in waterVolume.waterMeshes)
            {
                go.BroadcastMessage("SetWaveScale", waveScaleSlider.value, SendMessageOptions.DontRequireReceiver);
            }

          
        }

        // Update is called once per frame
        void Update()
        {
            itemCountText.text = waterVolume.statistics.itemCount.ToString("G");
            fpsText.text = waterVolume.statistics.fps.ToString("G");

        }

        public void OnWebLink(){
            Application.OpenURL("https://assetstore.unity.com/packages/slug/41495");
        }
   public void OnDownloadLink(){
            Application.OpenURL("http://popcannibal.com/watervolume/");
        }

        public void AddPlank()
        {
            AddGameObject(plankPrefab);//.transform.Rotate(new Vector3(0, 0, 45));

        }

        public void AddPontoonBoat()
        {
            AddGameObject(pontoonBoatPrefab, 8);
        }

        public void AddCubes()
        {
            AddGameObject(cubePrefab, 1);
        }

         public void AddBigCubes()
        {
            AddGameObject(bigCubePrefab, 4);
        }


        public void AddLongPlanks()
        {
            AddGameObject(longPlankPrefab, 16);
        }


        public void AddCylinder()
        {
            AddGameObject(cylinderPrefab);
        }

        public void Reset()
        {
            foreach (GameObject go in testObjects)
            {
                Destroy(go);
            }

            testObjects.Clear();
        }

        private GameObject AddGameObject(GameObject prefab, int offset = 2)
        {
            Color color = new Color(Random.Range(0.5f, 1), Random.Range(0.5f, 1), Random.Range(0.5f, 1));
            int count = System.Convert.ToInt32(spawnCount.text);
            GameObject go = null;

            int droppedCount = 0;

            int maxDimension = 38;

            int itemsPerRow = maxDimension / offset;
            int countPerLayer = itemsPerRow * itemsPerRow;



            int zOffset = 0;

            while (droppedCount < count)
            {

                if (countPerLayer >= count)
                {
                    countPerLayer = count;
                    itemsPerRow = (int)Mathf.Sqrt(count);
                }

                for (int x = 0; x <= itemsPerRow; x++)
                {
                    for (int y = 0; y <= itemsPerRow; y++)
                    {

                        go = (GameObject)GameObject.Instantiate(prefab, spawnPoint.transform.position +
                                                                        Vector3.right * x * offset + (Vector3.left * itemsPerRow * offset / 2f) +
                                                                        Vector3.back * y * offset + (Vector3.forward * itemsPerRow * offset / 2f) +
                                                                        Vector3.up * zOffset,
                                                                        Quaternion.identity);
                        testObjects.Add(go);

                        if (go.GetComponent<Rigidbody>() != null)
                        {
                            go.GetComponent<Rigidbody>().velocity = spawnVelocity;
                            go.GetComponent<Renderer>().material.SetColor("_Color", color);

                        }
                        go.name += " " + Time.realtimeSinceStartup;
                        droppedCount++;
                        if (droppedCount == count)
                        {
                            break;
                        }
                    }
                    if (droppedCount == count)
                    {
                        break;
                    }
                }
                zOffset += offset;
            }
            return go;
        }


        public void OnWaveScaleSlider(float value)
        {
            foreach (GameObject go in waterVolume.waterMeshes)
            {
                go.BroadcastMessage("SetWaveScale", value, SendMessageOptions.DontRequireReceiver);
            }
            waveScaleText.text = value.ToString();
        }

        public void OnRaycastSpacingSlider(float value)
        {
            waterVolume.raycastSpacing = value;
            raySpacingText.text = waterVolume.raycastSpacing.ToString();
        }

        public void OnBuoyancySlider(float value)
        {
            waterVolume.buoyancy = value;
            buoyancyText.text = waterVolume.buoyancy.ToString();
        }

        public void OnViscositySlider(float value)
        {
            waterVolume.viscosity = value;
            viscosityText.text = waterVolume.viscosity.ToString();
        }

        public void OnWaveSpeedSlider(float value)
        {
            foreach (GameObject go in waterVolume.waterMeshes)
            {
                go.BroadcastMessage("SetWaveSpeed", value, SendMessageOptions.DontRequireReceiver);
            }
            waveSpeedText.text = value.ToString();
        }

        public void OnWaterFlowSlider(float value)
        {
            waterVolume.flow.z = value;
        }


        public void OnThreading(bool value)
        {
            waterVolume.useThreads = value;
        }

        public void OnDrafting(bool value)
        {
            waterVolume.simulateDrafting = value;
        }

        public void OnSplashes(bool value)
        {
            if (value)
            {
                splasher.TurnOn();
            }
            else
            {
                splasher.TurnOff();
            }
        }

        public void OnUseMeshes(bool value)
        {
            /*
            if (value)
            {
                waterVolume.waterBox.center = new Vector3(waterVolume.waterBox.center.x, waterVolume.waterBox.center.y+0.1f, waterVolume.waterBox.center.z);
            }
            else
            {
                waterVolume.waterBox.center = new Vector3(waterVolume.waterBox.center.x, waterVolume.waterBox.center.y - 0.1f, waterVolume.waterBox.center.z);

            }
          */

            waterVolume.useMeshesAsSurface = value;
            animatedWaterMesh.gameObject.SetActive(waterVolume.useMeshesAsSurface);
            staticWaterVisual.gameObject.SetActive(!waterVolume.useMeshesAsSurface);
        }

        public void OnExitButton(){
            Application.Quit();
        }

    }
}
