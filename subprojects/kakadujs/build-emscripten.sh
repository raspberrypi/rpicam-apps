#!/bin/sh

rm -rf build-wasm
mkdir -p build-wasm
#-DCMAKE_FIND_ROOT_PATH=/ is a workaround for find_path when run via EMSCRIPTEN emcmake
(cd build-wasm && emcmake cmake .. -DCMAKE_FIND_ROOT_PATH=/)
if [ $retVal -ne 0 ]; then
    echo "CMAKE FAILED"
    exit 1
fi

(cd build-wasm && emmake make VERBOSE=1 -j)
retVal=$?
if [ $retVal -ne 0 ]; then
    echo "MAKE FAILED"
    exit 1
fi
mkdir -p ./dist
cp ./build-wasm/src/kakadujs.js ./dist
cp ./build-wasm/src/kakadujs.wasm ./dist
(cd test/node; npm run test)
