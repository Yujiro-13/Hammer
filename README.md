# Hammer

## Layered directory

```
├── CMakeLists.txt
├── main
|   ├── include
|   |   ├── Driver
|   |   |   ├── adc.hpp
|   |   |   ├── AS5047P.hpp
|   |   |   ├── buzzer.hpp
|   |   |   ├── motor.hpp
|   |   |   ├── MPU6500.hpp
|   |   |   ├── PCA9632.hpp
|   |   |   ├── peripheral.hpp
|   |   |   └── sensor.hpp
|   |   ├── Micromouse
|   |   |   ├── Motion
|   |   |   |   ├── motion.hpp
|   |   |   |   └── adachi.hpp
|   |   |   └── UI
|   |   |       ├── fast.hpp
|   |   |       ├── lod.hpp
|   |   |       ├── search.hpp
|   |   |       ├── test.hpp
|   |   |       └── UI.hpp
|   |   ├── files.hpp
|   |   └── structs.hpp   
|   ├── adachi.cpp
|   ├── adc.cpp
|   ├── AS5047P.cpp
|   ├── buzzer.cpp
│   ├── CMakeLists.txt
|   ├── fast.cpp
|   ├── files.cpp
|   ├── interrupt.cpp
|   ├── log.cpp
|   ├── main.cpp
|   ├── micromouse.cpp
|   ├── motion.cpp
|   ├── motor.cpp
|   ├── MPU6500.cpp
|   ├── PCA9632.cpp
|   ├── search.cpp
|   ├── sensor.cpp
|   ├── test.cpp
│   └── UI.cpp
├── partitions.csv
├── sdkconfig
├── sdkconfig.old
└── README.md                  
```

### partitions.csv
```
nvs,data,nvs,0x9000,16K,
phy_init,data,phy,0xd000,4K,
logs,data,spiffs,0xe000,1M,
storage,data,fat,0x10e000,1M,
factory,app,factory,0x210000,1M,
```