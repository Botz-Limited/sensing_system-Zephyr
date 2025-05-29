cd /home/ee/sensing_fw
west build  --build-dir /home/ee/sensing_fw/build /home/ee/sensing_fw/ --board nrf5340dk/nrf5340/cpuapp --sysbuild
nrfjprog --program build/merged_CPUNET.hex  --verify --chiperase --reset
nrfjprog --program build/merged.hex  --verify --chiperase --reset