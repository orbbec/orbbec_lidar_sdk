# Orbbec Lidar SDK library
Orbbec Lidar SDK is a library for using Orbbec Lidar devices. Currently, it only supports the MS600 device.

## Getting Started

### Prerequisites

- CMake
- C++17
- PCL

### Getting the Code

```bash
git clone https://github.com/orbbec/orbbec_lidar_sdk.git
```

### Building the Code

```bash
mkdir build
cd build
cmake .. -DBUILD_EXAMPLES=ON
make
```

### Running the sample

```bash
./examples/scan_viewer/scan_viewer
```
