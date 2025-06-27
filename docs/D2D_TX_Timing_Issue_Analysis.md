# D2D TX Timing Issue Analysis

## Problem
When the mobile app sends commands to the primary device (via control service), the primary tries to forward them to the secondary device via D2D TX. However, if D2D TX discovery hasn't completed yet, these commands fail with "Service discovery not complete or handle not found".

## Affected Commands
All commands from mobile app to secondary device via primary:
1. Set time
2. Delete foot log
3. Delete BHI360 log
4. Delete activity log
5. Start activity
6. Stop activity
7. Trigger BHI360 calibration

## Current Behavior
- Commands fail with -EINVAL if D2D TX discovery isn't complete
- Control service logs an error but doesn't retry
- User must retry the command manually

## Solutions

### Option 1: Command Queue (Complex)
Queue commands until D2D TX discovery completes, then execute them.
- Pros: No commands lost
- Cons: Complex implementation, memory overhead

### Option 2: Retry with Delay (Simple)
If command fails with -EINVAL, retry after a delay.
- Pros: Simple to implement
- Cons: Delays command execution

### Option 3: Block Until Ready (Current)
Return error to mobile app, let user retry.
- Pros: Simple, no hidden delays
- Cons: Poor user experience

### Option 4: Delay Mobile App Connection (Best)
Don't advertise to mobile apps until D2D connection is fully established.
- Pros: Clean solution, no timing issues
- Cons: Slightly longer initial connection time

## Recommendation
The current implementation (Option 3) is actually reasonable because:
1. D2D TX discovery typically completes within 1-2 seconds
2. Users rarely send commands immediately after connection
3. The error is logged clearly
4. Commands can be retried

If this becomes a real issue, implement Option 4 by delaying mobile app advertisement until both D2D discoveries complete.