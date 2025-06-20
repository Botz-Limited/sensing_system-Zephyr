# Command Flow - Detailed Coda Version
The original sequence diagram shows:

## Coda Version Options
### Option 1: Single Flow (Current - Too Simple)

### Option 2: Two-Part Flow (More Detail)
#### Part A: Command Processing

#### Part B: Response Flow

### Option 3: Detailed Single Flow (Best Compromise)

### Option 4: Vertical Flow with More Steps

## Recommendation
Use **Option 3** or **Option 4** as they capture:
1. The command originates from the App
2. Primary receives and processes locally
3. Primary forwards to Secondary via D2D
4. Secondary executes the command
5. Secondary acknowledges back to Primary
6. Primary notifies the App with status
This maintains the essential flow while working within Coda limitations.
