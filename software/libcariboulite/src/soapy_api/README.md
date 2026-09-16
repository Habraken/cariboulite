# SoapySDR adapter

The parent [CMake build](../../CMakeLists.txt) creates `SoapyCariboulite` when
SoapySDR development files are found. This directory contains device discovery,
session ownership, configuration/sensors and RX/TX streaming implementations.

- `SoapyCariboulite.cpp`: registration and discovery entry points.
- `CaribouliteSession.cpp`: shared device session handling.
- `Cariboulite.cpp` and `CaribouliteSensors.cpp`: configuration and sensors.
- `CaribouliteStreamFunctions.cpp` and `CaribouliteStream.cpp`: stream operations.

Regression harnesses are in [tests](../../tests/), including
`test_soapy_stream.py`. An implemented TX API is not proof of successful
end-to-end GNU Radio transmission. Verified flowgraphs and application recipes
remain [DOC-07](../../../../roadmap.md#documentation-validation-backlog).
