# Orbbec Lidar SDK Library

The Orbbec Lidar SDK is a library designed for integrating and using Orbbec Lidar devices. Currently, the SDK supports only the MS600 device.

## Getting Started

### Prerequisites

Ensure that your environment meets the following requirements:

- **Operating System**: Ubuntu 20.04
- **Build System**: CMake
- **Programming Language**: C++17
- **Point Cloud Processing**: PCL (Point Cloud Library)

To install the necessary packages, run the following command:

```bash
sudo apt-get install build-essential libpcl-dev cmake
```

### Cloning the Repository

Download the Orbbec Lidar SDK code by cloning the GitHub repository:

```bash
git clone https://github.com/orbbec/orbbec_lidar_sdk.git
cd orbbec_lidar_sdk
```

### Building the SDK

Follow the steps below to build the SDK:

1. Create a build directory and navigate into it:

    ```bash
    mkdir build
    cd build
    ```

2. Run CMake to configure the build, enabling the examples:

    ```bash
    cmake .. -DBUILD_EXAMPLES=ON
    ```

3. Compile the code using `make`:

    ```bash
    make
    ```

### Running the Sample Application

Once the build is complete, you can run the sample application to test the Lidar SDK:

```bash
./examples/scan_viewer/scan_viewer
```

This will launch the `scan_viewer` application, which is an example program demonstrating the basic usage of the Orbbec Lidar SDK.
