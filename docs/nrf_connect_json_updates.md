# nRF Connect Custom GATT JSON Updates

## Summary of Changes

Updated the `nrf_connect_custom_gatt.json` file to include all new characteristics that the primary device sends and receives.

### Added to Information Service:

1. **Total Step Count** (`0c372ec4-27eb-437e-bef4-775aefaf3c97`)
   - Properties: read, notify
   - Description: Aggregated step count from both feet

2. **Activity Step Count** (`0c372ec5-27eb-437e-bef4-775aefaf3c97`)
   - Properties: read, notify
   - Description: Steps counted during current activity only

### Added D2D RX Service (new service):

**Service UUID**: `e060ca1f-3115-4ad6-9709-8c5ff3bf558b`
**Service Name**: D2D RX Service

This service allows the primary device to receive commands from the phone to relay to the secondary device.

Characteristics:
1. **D2D Set Time** (`e160ca1f-3115-4ad6-9709-8c5ff3bf558b`) - write
2. **D2D Delete Foot Log** (`e160ca82-3115-4ad6-9709-8c5ff3bf558b`) - write
3. **D2D Delete BHI360 Log** (`e160ca83-3115-4ad6-9709-8c5ff3bf558b`) - write
4. **D2D Start Activity** (`e160ca84-3115-4ad6-9709-8c5ff3bf558b`) - write
5. **D2D Stop Activity** (`e160ca85-3115-4ad6-9709-8c5ff3bf558b`) - write
6. **D2D FOTA Status** (`e160ca86-3115-4ad6-9709-8c5ff3bf558b`) - write
7. **D2D Trigger BHI360 Calibration** (`e160ca87-3115-4ad6-9709-8c5ff3bf558b`) - write
8. **D2D Delete Activity Log** (`e160ca88-3115-4ad6-9709-8c5ff3bf558b`) - write

## Usage in nRF Connect

1. Open nRF Connect for Mobile
2. Go to Settings → About → Tap version 10 times to enable developer mode
3. Go back to Settings → Developer options
4. Select "Custom GATT configuration"
5. Import the `nrf_connect_custom_gatt.json` file
6. Connect to the device - all characteristics will now show with proper names

## Note

The D2D TX Service (secondary device) is NOT included in this JSON as it's only present on secondary devices, and the primary device (which the phone connects to) doesn't have this service.