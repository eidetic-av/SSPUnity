using Eidetic.PointClouds;
using System;
using System.Collections.Generic;
using System.Reflection;
using System.Threading;
using System.Runtime.InteropServices;
using UnityEngine;
using UnityEngine.Rendering;
using static UnityEngine.Experimental.Rendering.GraphicsFormat;

namespace Eidetic.SensorStreamPipe
{
    public enum SensorType {
        AzureKinect, Kinect2
    };

    public class SSPClient : MonoBehaviour
    {
        public bool DebugLog = false;

        [DllImport("ssp_client_plugin")]
        static extern void InitSubscriber(string host, int port, int pollTimeoutMs, out int subscriberAddress);

        [DllImport("ssp_client_plugin")]
        static extern void Close(int subscriberAddress);

        [DllImport("ssp_client_plugin")]
        static extern bool GetNextFramePtrs(int subscriberAddress, out IntPtr depthFramePtr,
            out IntPtr colorFramePtr, out IntPtr jointDataPtr);

        public Vector2 ThresholdX = new Vector2(-10f, 10f);
        public Vector2 ThresholdY = new Vector2(-10f, 10f);
        public Vector2 ThresholdZ = new Vector2(0, 12f);

        public Vector3 Translation = Vector3.zero;
        public Vector3 Rotation = Vector3.zero;

        public PointCloud PointCloud;

        public bool Active;

        int FrameWidth;
        int FrameHeight;
        int FrameSize => FrameWidth * FrameHeight;

        ComputeBuffer DepthBuffer;
        ComputeBuffer Depth2DTo3DBuffer;
        ComputeShader DepthTransferShader;

        int SubscriberAddress;

        public void Init(string host, int hostPort, SensorType sensor)
        {          
            if (sensor is SensorType.AzureKinect) {
              FrameWidth = 640;
              FrameHeight = 576;
              Depth2DTo3DBuffer = LoadLookupTable("AZURE_NFOV_UNBINNED", FrameSize);
            }
            else if (sensor is SensorType.Kinect2) {
              FrameWidth = 512;
              FrameHeight = 424;
              Depth2DTo3DBuffer = LoadLookupTable("KINECT2", FrameSize);
            }
            // quit if we couldn't load a lookup table
            if (Depth2DTo3DBuffer == null) return;

            DepthTransferShader = Resources.Load("DepthTransfer") as ComputeShader;
            
            // depthbuffer length is halved because each 32-bit entry holds two shorts
            DepthBuffer = new ComputeBuffer(FrameSize / 2, 4);

            InitSubscriber(host, hostPort, 1, out SubscriberAddress);

            PointCloud = PointCloud.CreateInstance();
        }

        void Update() {
            if (Active) UpdateFrame();
        }
        
        void UpdateFrame()
        {
            if (PointCloud == null) return;

            var newFrame = GetNextFramePtrs(SubscriberAddress, out IntPtr depthFramePtr,
                out IntPtr colorFramePtr, out IntPtr jointDataPtr);
            if (!newFrame) return;
            
            Debug("Updating textures.");

            if (depthFramePtr != IntPtr.Zero)
            {
                // Set the depth buffer from the plugin memory location
                SetUnmanagedData(DepthBuffer, depthFramePtr, FrameSize / 2, 4);
            
                // allocate the data for the depth shader
                DepthTransferShader.SetInt("Width", FrameWidth);
                DepthTransferShader.SetInt("Height", FrameHeight);
                DepthTransferShader.SetVector("Translation", Translation); 
                DepthTransferShader.SetVector("Rotation", AsVector(Quaternion.Euler(Rotation))); 
                DepthTransferShader.SetVector("ThresholdX", ThresholdX); 
                DepthTransferShader.SetVector("ThresholdY", ThresholdY); 
                DepthTransferShader.SetVector("ThresholdZ", ThresholdZ); 
                DepthTransferShader.SetBuffer(0, "DepthBuffer", DepthBuffer);
                DepthTransferShader.SetBuffer(0, "Depth2DTo3DBuffer", Depth2DTo3DBuffer);

                // create the output texture
                var positionsRt = new RenderTexture(FrameWidth, FrameHeight, 24, R32G32B32A32_SFloat);
                positionsRt.enableRandomWrite = true;
                positionsRt.Create();
                DepthTransferShader.SetTexture(0, "Positions", positionsRt, 0);
        
                // dispatch the depth shader
                int gfxThreadWidth = FrameSize / 2 / 64;
                DepthTransferShader.Dispatch(0, gfxThreadWidth, 1, 1);
                // set the result to the pointcloud
                PointCloud.SetPositionMap(positionsRt);
                Debug("Set depth.");
            }
            
            if (colorFramePtr != IntPtr.Zero)
            {
                // create a color texture with the incoming colors
                var colorsTexture = new Texture2D(FrameWidth, FrameHeight, TextureFormat.RGBA32, false);
                colorsTexture.LoadRawTextureData(colorFramePtr, FrameSize * 4);
                colorsTexture.Apply();
                // // set the results to the pointcloud
                PointCloud.SetColorMap(colorsTexture);
                Debug("Set colour.");
            }

            if (jointDataPtr != IntPtr.Zero)
            {
                // var detectedBodies = 1;
                // var skeletonSize = Marshal.SizeOf(typeof(Skeleton));
                // var skeletons = new Skeleton[detectedBodies];
                // for (int i = 0; i < detectedBodies; i++)
                // {
                // }
                // float[] hip = new float[3];
                // Marshal.Copy(jointDataPtr, hip, 0, 3);
                // Debug($"Got hip {hip[0]}, {hip[1]}, {hip[2]}");
            }
        }

        void OnDestroy()
        {
            Active = false;
            DepthBuffer?.Release();
            Close(SubscriberAddress);
        }

        // This function expects a lookup table asset as a space delimited file
        // where each entry contains comma delimited x y scalars
        ComputeBuffer LoadLookupTable(string fileName, int frameSize) {
            var lutAsset = Resources.Load(fileName) as TextAsset;
            if (lutAsset == null) {
               Debug("Unable to load 2D to 3D lookup table.");
               return null;
            }

            Debug($"Loaded lookup table asset {fileName}.");

            var lookupTable = new List<Vector2>();
            var lutString = lutAsset.text;
            var lutEntries = lutString.Split(" ".ToCharArray(), StringSplitOptions.RemoveEmptyEntries);
            var entryCount = 0;

            foreach(var entry in lutEntries)
            {
                var vector = entry.Split(",".ToCharArray());
                if (vector.Length < 2) continue;

                var x = float.Parse(vector[0]);
                var y = float.Parse(vector[1]);

                lookupTable.Add(new Vector2(x, y));
                entryCount++;
            }

            Debug($"Added {entryCount} values into the lookup table.");

            var lookupBuffer = new ComputeBuffer(FrameSize, sizeof(float) * 2);
            lookupBuffer.SetData(lookupTable);

            return lookupBuffer;
        }


        void Debug(string message)
        {
            if (DebugLog) UnityEngine.Debug.Log("SSP_DEBUG: " + message);
        }

        static Vector4 AsVector(Quaternion quat) => new Vector4(quat.x, quat.y, quat.z, quat.w);

        static MethodInfo SetNativeMethod;
        static object [] MethodArgs = new object[5];

        public static void SetUnmanagedData
            (ComputeBuffer buffer, IntPtr pointer, int count, int stride, int srcOffset = 0, int bufferOffset =0)
        {
            if (SetNativeMethod == null)
            {
                SetNativeMethod = typeof(ComputeBuffer).GetMethod(
                    "InternalSetNativeData",
                    BindingFlags.InvokeMethod |
                    BindingFlags.NonPublic |
                    BindingFlags.Instance
                );
            }

            MethodArgs[0] = pointer;
            MethodArgs[1] = srcOffset;
            MethodArgs[2] = bufferOffset;
            MethodArgs[3] = count;
            MethodArgs[4] = stride;

            SetNativeMethod.Invoke(buffer, MethodArgs);
        }

        struct Skeleton {
            public float HipX;
            public float HipY;
            public float HipZ;
        }
    }
    
}

