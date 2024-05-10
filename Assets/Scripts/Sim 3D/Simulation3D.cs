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
    float deltaTime;
    int numParticles;
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
        // SimulationFunctions.ExternalForces(positionBuffer, velocityBuffer, numParticles, gravity, Time.fixedDeltaTime);
        // SimulationFunctions.UpdateSpatialHash(positionBuffer, predictedPositionsBuffer, spatialIndices, spatialOffsets, numParticles, smoothingRadius);
        // SimulationFunctions.CalculateDensities(predictedPositionsBuffer, densityBuffer, spatialIndices, spatialOffsets, numParticles, smoothingRadius);
        // SimulationFunctions.CalculatePressureForce(predictedPositionsBuffer, velocityBuffer, densityBuffer, spatialIndices, spatialOffsets, numParticles, smoothingRadius, targetDensity, pressureMultiplier, nearPressureMultiplier);
        // SimulationFunctions.CalculateViscosity(predictedPositionsBuffer, velocityBuffer, spatialIndices, spatialOffsets, numParticles, smoothingRadius, viscosityStrength);
        // SimulationFunctions.UpdatePositions(positionBuffer, predictedPositionsBuffer, velocityBuffer, numParticles, Time.fixedDeltaTime, collisionDamping);
    }

    void UpdateSettings(float deltaTime)
    {
        Vector3 simBoundsSize = transform.localScale;
        Vector3 simBoundsCentre = transform.position;

        SimulationFunctions.SetSettings(deltaTime, gravity, collisionDamping,
                                         smoothingRadius, targetDensity, pressureMultiplier,
                                         nearPressureMultiplier, viscosityStrength, simBoundsSize,
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
        // Draw Bounds
        var m = Gizmos.matrix;
        Gizmos.matrix = transform.localToWorldMatrix;
        Gizmos.color = new Color(0, 1, 0, 0.5f);
        Gizmos.DrawWireCube(Vector3.zero, Vector3.one);
        Gizmos.matrix = m;
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
