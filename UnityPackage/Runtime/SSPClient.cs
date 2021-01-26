using Eidetic.PointClouds;
using System;
using System.Collections.Generic;
using System.Reflection;
using System.Threading;
using System.Runtime.InteropServices;
using UnityEngine;
using static UnityEngine.Experimental.Rendering.GraphicsFormat;

namespace Eidetic.SensorStreamPipe
{
    public class SSPClient : MonoBehaviour
    {
        public string Host = "localhost";
        public int HostPort = 9999;

        public string LutName = "NFOV_UNBINNED";
        public bool DebugLog = true;

        [DllImport("ssp_client_plugin")]
        static extern void InitSubscriber(string host, int port, int pollTimeoutMs);

        [DllImport("ssp_client_plugin")]
        static extern void Close();

        [DllImport("ssp_client_plugin")]
        static extern bool GetNextFramePtrs(out IntPtr depthFramePtr, out IntPtr colorFramePtr);

        const int DepthFrameWidth = 640;
        const int DepthFrameHeight = 576;
        const int DepthFrameSize = DepthFrameWidth * DepthFrameHeight;

        const int ColorFrameWidth = 1280;
        const int ColorFrameHeight = 720;
        const int ColorFrameSize = ColorFrameWidth * ColorFrameHeight;

        public Vector2 ThresholdX = new Vector2(-10f, 10f);
        public Vector2 ThresholdY = new Vector2(-10f, 10f);
        public Vector2 ThresholdZ = new Vector2(0, 12f);

        public Vector3 Translation = Vector3.zero;
        public Vector3 Rotation = Vector3.zero;

        public PointCloud PointCloud;

        ComputeBuffer DepthBuffer;
        ComputeBuffer Depth2DTo3DBuffer;
        ComputeShader DepthTransferShader;

        bool ClientActive;
        bool DispatchUpdate;
    
        void Start()
        {
            InitSubscriber(Host, HostPort, 1);
            Debug("Initialised Subscriber");

            PointCloud = PointCloud.CreateInstance();
        
            DepthTransferShader = Resources.Load("DepthTransfer") as ComputeShader;
            // depthbuffer length is halved because each 32-bit entry holds two shorts
            DepthBuffer = new ComputeBuffer(DepthFrameSize / 2, 4);

            var lutAsset = Resources.Load(LutName) as TextAsset;
            if (lutAsset == null)
            {
                Debug("Unable to load 2D to 3D lookup table.");
                return;
            }
            Debug($"Loaded lookup table asset {LutName}.");

            var lookupTable = new List<Vector2>();
            var lutString = lutAsset.text;
            var lutEntries = lutString.Split(" ".ToCharArray(), StringSplitOptions.RemoveEmptyEntries);
            var i = 0;

            foreach(var entry in lutEntries)
            {
                var vector = entry.Split(",".ToCharArray());
                if (vector.Length < 2) continue;

                var x = float.Parse(vector[0]);
                var y = float.Parse(vector[1]);

                lookupTable.Add(new Vector2(x, y));
                i++;
            }

            Debug($"Added {i} entries into the lookup table.");

            Depth2DTo3DBuffer = new ComputeBuffer(DepthFrameSize, sizeof(float) * 2);
            Depth2DTo3DBuffer.SetData(lookupTable);
        
            DepthTransferShader.SetBuffer(0, "Depth2DTo3DBuffer", Depth2DTo3DBuffer);
        }

        void OnDestroy()
        {
            ClientActive = false;
            DepthBuffer.Release();
            Close();
        }

        void Update()
        {
            var newFrame = GetNextFramePtrs(out IntPtr depthFramePtr, out IntPtr colorFramePtr);
            if (!newFrame) return;
            Debug("Updating textures.");

            if (depthFramePtr != IntPtr.Zero)
            {
                // Set the depth buffer from the plugin memory location
                SetUnmanagedData(DepthBuffer, depthFramePtr, DepthFrameSize / 2, 4);
            
                // allocate the data for the depth shader
                DepthTransferShader.SetInt("Width", DepthFrameWidth);
                DepthTransferShader.SetInt("Height", DepthFrameHeight);
                DepthTransferShader.SetVector("ThresholdX", ThresholdX); 
                DepthTransferShader.SetVector("ThresholdY", ThresholdY); 
                DepthTransferShader.SetVector("ThresholdZ", ThresholdZ); 
                DepthTransferShader.SetBuffer(0, "DepthBuffer", DepthBuffer);

                // create the output texture
                var positionsRt = new RenderTexture(DepthFrameWidth, DepthFrameHeight, 24, R32G32B32A32_SFloat);
                positionsRt.enableRandomWrite = true;
                positionsRt.Create();
                DepthTransferShader.SetTexture(0, "Positions", positionsRt, 0);
        
                // dispatch the depth shader
                int gfxThreadWidth = DepthFrameSize / 2 / 64;
                DepthTransferShader.Dispatch(0, gfxThreadWidth, 1, 1);
                // set the result to the pointcloud
                PointCloud.SetPositionMap(positionsRt);
                Debug("Set depth.");
            }
            
            if (colorFramePtr != IntPtr.Zero)
            {
                // create a color texture with the incoming colors
                var colorsTexture = new Texture2D(DepthFrameWidth, DepthFrameHeight, TextureFormat.RGBA32, false);
                colorsTexture.LoadRawTextureData(colorFramePtr, DepthFrameSize * 4);
                colorsTexture.Apply();
                // // set the results to the pointcloud
                PointCloud.SetColorMap(colorsTexture);
                Debug("Set colour.");
            }
        }

        void Debug(string message)
        {
            if (DebugLog) UnityEngine.Debug.Log("SSP_DEBUG: " + message);
        }

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
    }
    
}

