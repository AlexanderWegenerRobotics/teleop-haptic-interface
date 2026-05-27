# teleop-haptic-interface

cmake .. -DWITH_SIGMA=OFF
cmake --build . --config Release

:: With Sigma SDK:
cmake .. -DWITH_SIGMA=ON
cmake --build . --config Release