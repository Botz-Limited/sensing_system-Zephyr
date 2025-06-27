# Version Management Best Practice

## Recommended Approach: Single Source of Truth

Use the VERSION file as the single source of truth, as it's the Zephyr standard.

### Benefits:
1. **Zephyr standard** - VERSION file is how Zephyr manages versions
2. **Automatic generation** - app_version.h is generated from VERSION file
3. **Consistent** - All version info comes from one place
4. **MCUboot compatible** - Works with firmware updates

### Implementation:

1. **Update VERSION file** when releasing:
   ```
   VERSION_MAJOR = 1
   VERSION_MINOR = 0
   PATCHLEVEL = 0
   VERSION_TWEAK = 0
   EXTRAVERSION = 
   ```

2. **Remove hardcoded version from CMakeLists.txt**:
   ```cmake
   # Remove this line:
   # set(CONFIG_APP_VERSION "1.0.0")
   
   # Fix MCUboot version to use Zephyr's version:
   set(CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION "${APPLICATION_VERSION_STRING}")
   ```

3. **Use generated macros everywhere**:
   - `APP_VERSION_STRING` - "1.0.0"
   - `APP_VERSION_MAJOR` - 1
   - `APP_VERSION_MINOR` - 0
   - `APP_PATCHLEVEL` - 0

### Current Usage:
- Device Info Service ✓ (already uses APP_VERSION_MAJOR/MINOR/PATCHLEVEL)
- Boot banner ✓ (uses APP_VERSION_STRING)
- MCUboot signing ✗ (needs fixing)

### Version Format:
- Development: "1.0.0-dev" (set EXTRAVERSION = -dev)
- Release: "1.0.0" (clear EXTRAVERSION)
- With tweak: "1.0.0+5" (set VERSION_TWEAK = 5)