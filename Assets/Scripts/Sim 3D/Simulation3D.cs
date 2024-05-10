using UnityEngine;
using Unity.Mathematics;

public class Simulation3D : MonoBehaviour
{
    public event System.Action SimulationStepCompleted;

    [Header("Settings")]
    public float timeScale = 1;
    private bool fixedTimeStep = true;
    public int iterationsPerFrame;
    public float gravity = -10;
    [Range(0, 1)] public float collisionDamping = 0.05f;
    public float smoothingRadius = 0.2f;
    public float targetDensity;
    public float pressureMultiplier;
    public float nearPressureMultiplier;
    public float viscosityStrength;

    private Vector3 BoundsSize = new Vector3(1, 1, 1);

    [Header("References")]
    public Spawner3D spawner;
    public ParticleDisplay3D display;

    // Buffers
    public ComputeBuffer positionBuffer { get; private set; }
    public ComputeBuffer velocityBuffer { get; private set; }
    public ComputeBuffer predictedPositionsBuffer;
    ComputeBuffer spatialIndices;
    ComputeBuffer spatialOffsets;
    ComputeBuffer densityBuffer;

    // State
    bool isPaused;
    bool pauseNextFrame;
    Spawner3D.SpawnData spawnData;
    private float deltaTime;
    private int numParticles;


    private Vector3[] obstacleCentres = new Vector3[4];
    private Vector3[] obstacleSizes = new Vector3[4];
    private float[] obstacleRadii = new float[4];
    private float[] obstacleHeights = new float[4];
    void Start()
    {
        Debug.Log("Controls: Space = Play/Pause, R = Reset");
        Debug.Log("Use transform tool in scene to scale/rotate simulation bounding box.");

        deltaTime = 1 / 60f;
        Time.fixedDeltaTime = deltaTime;

        spawnData = spawner.GetSpawnData();

        // Create buffers
        numParticles = spawnData.points.Length;
        positionBuffer = CreateStructuredBuffer<float3>(numParticles);
        predictedPositionsBuffer = CreateStructuredBuffer<float3>(numParticles);
        velocityBuffer = CreateStructuredBuffer<float3>(numParticles);
        densityBuffer = CreateStructuredBuffer<float2>(numParticles);
        spatialIndices = CreateStructuredBuffer<uint3>(numParticles);
        spatialOffsets = CreateStructuredBuffer<uint>(numParticles);

        // Set buffer data
        SetInitialBufferData(spawnData);

        // Init display
        display.Init(this);
    }

    void FixedUpdate()
    {
        // Run simulation if in fixed timestep mode
        if (fixedTimeStep)
        {
            RunSimulationFrame(Time.fixedDeltaTime);
        }
    }

    void Update()
    {
        // Run simulation if not in fixed timestep mode
        // (skip running for first few frames as timestep can be a lot higher than usual)
        if (!fixedTimeStep && Time.frameCount > 10)
        {
            RunSimulationFrame(Time.deltaTime);
        }

        if (pauseNextFrame)
        {
            isPaused = true;
            pauseNextFrame = false;
        }

        HandleInput();
    }

    void RunSimulationFrame(float frameTime)
    {
        if (!isPaused)
        {
            float timeStep = frameTime / iterationsPerFrame * timeScale;

            UpdateSettings(timeStep);

            for (int i = 0; i < iterationsPerFrame; i++)
            {
                RunSimulationStep();
                SimulationStepCompleted?.Invoke();
            }
        }
    }

    void RunSimulationStep()
    {
        SimulationFunctions.ExternalForces(positionBuffer, velocityBuffer, numParticles, gravity, Time.fixedDeltaTime);
        SimulationFunctions.UpdateSpatialHash(positionBuffer, predictedPositionsBuffer, spatialIndices, spatialOffsets, numParticles, smoothingRadius);
        SimulationFunctions.CalculateDensities(predictedPositionsBuffer, densityBuffer, spatialIndices, spatialOffsets, numParticles, smoothingRadius);
        SimulationFunctions.CalculatePressureForce(predictedPositionsBuffer, velocityBuffer, densityBuffer, spatialIndices, spatialOffsets, numParticles, smoothingRadius, targetDensity, pressureMultiplier, nearPressureMultiplier, Time.fixedDeltaTime);
        SimulationFunctions.CalculateViscosity(predictedPositionsBuffer, velocityBuffer, spatialIndices, spatialOffsets, numParticles, smoothingRadius, viscosityStrength);
        SimulationFunctions.UpdatePositions(positionBuffer, predictedPositionsBuffer, velocityBuffer, numParticles, Time.fixedDeltaTime, collisionDamping);
        SimulationFunctions.ResolveCollisions(collisionDamping, positionBuffer, velocityBuffer, numParticles, obstacleCentres, obstacleSizes, obstacleRadii, obstacleHeights);
    }

    void UpdateSettings(float deltaTime)
    {
        Vector3 simBoundsSize = transform.localScale;
        Vector3 simBoundsCentre = transform.position;

        SimulationFunctions.SetSettings(deltaTime, gravity, collisionDamping,
                                         smoothingRadius, targetDensity, pressureMultiplier,
                                         nearPressureMultiplier, viscosityStrength, BoundsSize,
                                         simBoundsCentre, transform.localToWorldMatrix, transform.worldToLocalMatrix);
    }

    void SetInitialBufferData(Spawner3D.SpawnData spawnData)
    {
        positionBuffer.SetData(spawnData.points);
        predictedPositionsBuffer.SetData(spawnData.points);
        velocityBuffer.SetData(spawnData.velocities);
    }

    void HandleInput()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            isPaused = !isPaused;
        }

        if (Input.GetKeyDown(KeyCode.RightArrow))
        {
            isPaused = false;
            pauseNextFrame = true;
        }

        if (Input.GetKeyDown(KeyCode.R))
        {
            isPaused = true;
            SetInitialBufferData(spawnData);
        }
    }

    void OnDestroy()
    {
        positionBuffer.Release();
        predictedPositionsBuffer.Release();
        velocityBuffer.Release();
        densityBuffer.Release();
        spatialIndices.Release();
        spatialOffsets.Release();
    }

    void OnDrawGizmos()
    {
        for (int i = 0; i < 4; i++)
        {
            obstacleCentres[i] = new Vector3(i * 3 - 4.5f, i*0.9f - 1.5f, 0) * 0.1f;
            obstacleSizes[i] = new Vector3(5, 1, 10) * 0.1f;
            obstacleRadii[i] = 0.1f;
            obstacleHeights[i] = 0.4f;
        }
        // Draw Bounds
        var m = Gizmos.matrix;
        Gizmos.matrix = transform.localToWorldMatrix;
        Gizmos.color = new Color(0, 1, 0, 0.5f);
        Gizmos.DrawWireCube(transform.position, BoundsSize);
        for (int i = 0; i < obstacleCentres.Length; i++)
        {
            Gizmos.color = new Color(1, 0, 0, 0.5f);
            Gizmos.DrawCube(obstacleCentres[i], obstacleSizes[i]);
            // Gizmos.DrawSphere(obstacleCentres[i], obstacleRadii[i]);
            // DrawRotatedBox(obstacleCentres[i], obstacleSizes[i]);
            // DrawWireCylinder(obstacleCentres[i], obstacleRadii[i], obstacleHeights[i], 36);
        }
        Gizmos.matrix = m;
    }
    void DrawWireCylinder(Vector3 Center, float radius, float height, int segments)
    {
        // Calculate the angle increment between each segment
        float angleIncrement = 360f / segments;
        float half_height = 0.5f * height;

        // Loop through each segment to draw the cylinder
        for (int i = 0; i < segments; i++)
        {
            // Calculate the angle for the current segment
            float angle = Mathf.Deg2Rad * i * angleIncrement;

            // Calculate the positions of the vertices for the current segment
            Vector3 point1 = new Vector3(radius * Mathf.Cos(angle), -half_height, radius * Mathf.Sin(angle));
            Vector3 point2 = new Vector3(radius * Mathf.Cos(angle + Mathf.Deg2Rad * angleIncrement), -half_height, radius * Mathf.Sin(angle + Mathf.Deg2Rad * angleIncrement));
            Vector3 point3 = new Vector3(point2.x, half_height, point2.z);
            Vector3 point4 = new Vector3(point1.x, half_height, point1.z);

            // Apply rotation to each point
            point1 = 1 * point1 + Center;
            point2 = 1 * point2 + Center;
            point3 = 1 * point3 + Center;
            point4 = 1 * point4 + Center;

            // Draw the lines connecting the vertices to form the wireframe
            Gizmos.DrawLine(point1, point2);
            Gizmos.DrawLine(point2, point3);
            Gizmos.DrawLine(point3, point4);
            Gizmos.DrawLine(point4, point1);
            Vector3 bottom = new Vector3(0, -half_height, 0);
            Vector3 top = new Vector3(0, half_height, 0);

            bottom = 1 * bottom + Center;
            top = 1 * top + Center;
            // Draw the lines from Center to top and vice versa
            Gizmos.DrawLine(bottom, point1);
            Gizmos.DrawLine(top, point4);
        }
    }

    void DrawRotatedBox(Vector3 obstacleCentres, Vector3 obstacleSizes)
    {

        // Define cube vertices
        Vector3[] cubeVertices = new Vector3[]
        {
            new Vector3(-obstacleSizes.x * 0.5f, -obstacleSizes.y * 0.5f, -obstacleSizes.z * 0.5f), // Bottom back left
            new Vector3(obstacleSizes.x * 0.5f, -obstacleSizes.y * 0.5f, -obstacleSizes.z * 0.5f), // Bottom back right
            new Vector3(obstacleSizes.x * 0.5f, -obstacleSizes.y * 0.5f, obstacleSizes.z * 0.5f), // Bottom front right
            new Vector3(-obstacleSizes.x * 0.5f, -obstacleSizes.y * 0.5f, obstacleSizes.z * 0.5f), // Bottom front left
            new Vector3(-obstacleSizes.x * 0.5f, obstacleSizes.y * 0.5f, -obstacleSizes.z * 0.5f), // Top back left
            new Vector3(obstacleSizes.x * 0.5f, obstacleSizes.y * 0.5f, -obstacleSizes.z * 0.5f), // Top back right
            new Vector3(obstacleSizes.x * 0.5f, obstacleSizes.y * 0.5f, obstacleSizes.z * 0.5f), // Top front right
            new Vector3(-obstacleSizes.x * 0.5f, obstacleSizes.y * 0.5f, obstacleSizes.z * 0.5f) // Top front left
        };
        // Apply rotation to each vertex
        for (int j = 0; j < cubeVertices.Length; j++)
        {
            cubeVertices[j] = 1 * cubeVertices[j] + obstacleCentres;
        }
        Gizmos.DrawLine(cubeVertices[0], cubeVertices[1]);
        Gizmos.DrawLine(cubeVertices[1], cubeVertices[2]);
        Gizmos.DrawLine(cubeVertices[2], cubeVertices[3]);
        Gizmos.DrawLine(cubeVertices[3], cubeVertices[0]);
        Gizmos.DrawLine(cubeVertices[4], cubeVertices[5]);
        Gizmos.DrawLine(cubeVertices[5], cubeVertices[6]);
        Gizmos.DrawLine(cubeVertices[6], cubeVertices[7]);
        Gizmos.DrawLine(cubeVertices[7], cubeVertices[4]);
        Gizmos.DrawLine(cubeVertices[0], cubeVertices[4]);
        Gizmos.DrawLine(cubeVertices[1], cubeVertices[5]);
        Gizmos.DrawLine(cubeVertices[2], cubeVertices[6]);
        Gizmos.DrawLine(cubeVertices[3], cubeVertices[7]);
    }

    public static ComputeBuffer CreateStructuredBuffer<T>(int count)
    {
        return new ComputeBuffer(count, GetStride<T>());
    }

    public static int GetStride<T>()
    {
        return System.Runtime.InteropServices.Marshal.SizeOf(typeof(T));
    }

}
