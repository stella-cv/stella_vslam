# Stella VSLAM - System Architecture

## High-Level Architecture

```mermaid
graph TB
    subgraph "Stella VSLAM System"
        subgraph "Core Modules"
            TM[Tracking Module]
            MM[Mapping Module]
            GOM[Global Optimisation Module]
        end
        
        subgraph "Data Management"
            DB[(Map Database)]
            BOW[(BoW Database)]
            CAM[(Camera Database)]
        end
        
        subgraph "Processing"
            FE[Feature Extraction]
            CM[Camera Models]
            IO[I/O Modules]
        end
        
        subgraph "Publishing"
            MP[Map Publisher]
            FP[Frame Publisher]
        end
    end
    
    TM --> DB
    MM --> DB
    GOM --> DB
    TM --> BOW
    TM --> CAM
    FE --> TM
    CM --> TM
    IO --> DB
    MP --> DB
    FP --> TM
```

## Multi-Threaded Architecture

```mermaid
graph LR
    subgraph "Main Thread"
        UI[User Interface]
        CF[Camera Feed]
    end
    
    subgraph "Tracking Thread"
        TF[Frame Tracking]
        TI[Initialisation]
        TR[Relocalisation]
    end
    
    subgraph "Mapping Thread"
        KF[Keyframe Insertion]
        LM[Local Mapping]
        BA[Bundle Adjustment]
    end
    
    subgraph "Global Optimisation Thread"
        LD[Loop Detection]
        LC[Loop Closure]
        PGO[Pose Graph Optimisation]
    end
    
    UI --> CF
    CF --> TF
    TF --> KF
    KF --> LD
    LD --> LC
    LC --> PGO
    PGO --> TF
```

## Data Flow Architecture

```mermaid
flowchart TD
    A[Camera Input] --> B[Frame Creation]
    B --> C[Feature Extraction]
    C --> D[Tracking Module]
    D --> E{Tracking Success?}
    E -->|Yes| F[Pose Estimation]
    E -->|No| G[Relocalisation]
    G --> D
    F --> H[Keyframe Decision]
    H -->|Yes| I[Keyframe Insertion]
    H -->|No| J[Update Motion Model]
    I --> K[Local Mapping]
    K --> L[Bundle Adjustment]
    L --> M[Loop Detection]
    M -->|Loop Found| N[Loop Closure]
    M -->|No Loop| O[Continue]
    N --> P[Global Optimisation]
    P --> Q[Map Update]
    Q --> R[Pose Refinement]
    R --> D
    O --> D
    J --> D
```

## Component Interaction Diagram

```mermaid
sequenceDiagram
    participant User
    participant System
    participant Tracking
    participant Mapping
    participant GlobalOpt
    participant Database
    
    User->>System: Start SLAM
    System->>Tracking: Initialise
    System->>Mapping: Start mapping thread
    System->>GlobalOpt: Start optimisation thread
    
    loop Frame Processing
        User->>System: Feed frame
        System->>Tracking: Process frame
        Tracking->>Database: Query landmarks
        Tracking->>Tracking: Estimate pose
        
        alt New keyframe needed
            Tracking->>Mapping: Insert keyframe
            Mapping->>Database: Store keyframe
            Mapping->>Mapping: Local bundle adjustment
        end
        
        alt Loop detected
            Mapping->>GlobalOpt: Request loop closure
            GlobalOpt->>Database: Optimise pose graph
            GlobalOpt->>Tracking: Update poses
        end
    end
    
    User->>System: Save map
    System->>Database: Export map data
```

## Camera Model Architecture

```mermaid
classDiagram
    class CameraBase {
        <<abstract>>
        +project(points_3d)
        +unproject(points_2d)
        +get_camera_matrix()
    }
    
    class PerspectiveCamera {
        +fx, fy: focal lengths
        +cx, cy: principal point
        +distortion coefficients
    }
    
    class FisheyeCamera {
        +fx, fy: focal lengths
        +cx, cy: principal point
        +k1, k2, k3, k4: fisheye coefficients
    }
    
    class EquirectangularCamera {
        +cols, rows: image dimensions
        +fps: frame rate
    }
    
    class RadialDivisionCamera {
        +fx, fy: focal lengths
        +cx, cy: principal point
        +division coefficients
    }
    
    CameraBase <|-- PerspectiveCamera
    CameraBase <|-- FisheyeCamera
    CameraBase <|-- EquirectangularCamera
    CameraBase <|-- RadialDivisionCamera
```

## Database Schema

```mermaid
erDiagram
    KEYFRAME {
        unsigned_int id PK
        Mat44_t pose_cw
        timestamp timestamp
        vector landmarks
        vector observations
    }
    
    LANDMARK {
        unsigned_int id PK
        Vec3_t pos_w
        vector observations
        bool is_observable
    }
    
    OBSERVATION {
        unsigned_int keyframe_id FK
        unsigned_int landmark_id FK
        Vec2_t pos_2d
        float scale_level
    }
    
    CAMERA {
        unsigned_int id PK
        string name
        string model
        json parameters
    }
    
    KEYFRAME ||--o{ OBSERVATION : "has"
    LANDMARK ||--o{ OBSERVATION : "observed_by"
    CAMERA ||--o{ KEYFRAME : "captured_by"
```

## Module Dependencies

```mermaid
graph TD
    subgraph "Core System"
        SYS[System]
        CFG[Config]
    end
    
    subgraph "Tracking"
        TRK[Tracking Module]
        INIT[Initializer]
        REL[Relocalizer]
        KFI[Keyframe Inserter]
    end
    
    subgraph "Mapping"
        MAP[Mapping Module]
        LMC[Local Map Cleaner]
        LMU[Local Map Updater]
    end
    
    subgraph "Optimisation"
        GOM[Global Optimisation]
        LBA[Loop Bundle Adjuster]
        LD[Loop Detector]
    end
    
    subgraph "Data"
        MDB[Map Database]
        BDB[BoW Database]
        CDB[Camera Database]
    end
    
    subgraph "Features"
        FE[Feature Extraction]
        ORB[ORB Extractor]
    end
    
    SYS --> CFG
    SYS --> TRK
    SYS --> MAP
    SYS --> GOM
    
    TRK --> INIT
    TRK --> REL
    TRK --> KFI
    TRK --> MDB
    TRK --> BDB
    
    MAP --> LMC
    MAP --> LMU
    MAP --> MDB
    
    GOM --> LBA
    GOM --> LD
    GOM --> MDB
    
    TRK --> FE
    FE --> ORB
```

## System State Machine

```mermaid
stateDiagram-v2
    [*] --> Initialising
    Initialising --> Tracking : Initialisation Success
    Initialising --> Lost : Initialisation Failed
    
    Tracking --> Tracking : Continue Tracking
    Tracking --> Lost : Tracking Failed
    Tracking --> Mapping : New Keyframe
    
    Lost --> Tracking : Relocalisation Success
    Lost --> Lost : Relocalisation Failed
    
    Mapping --> Tracking : Mapping Complete
    Mapping --> GlobalOptimisation : Loop Detected
    
    GlobalOptimisation --> Tracking : Optimisation Complete
    
    Tracking --> [*] : System Shutdown
    Lost --> [*] : System Shutdown
    Mapping --> [*] : System Shutdown
    GlobalOptimisation --> [*] : System Shutdown
``` 