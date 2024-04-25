rm -rf build
PICO_BOARD=pico
export PICO_SDK_PATH='/home/kostas/Desktop/Pico-SDK/pico-sdk/'
mkdir build
cd build
cmake .. -DPICO_BOARD=$PICO_BOARD
make hermes_vi