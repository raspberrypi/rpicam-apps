// Copyright (c) Chris Hafey.
// SPDX-License-Identifier: MIT

const openjphjs = require('../../dist/kakadujs.js');
const fs = require('fs')

openjphjs.onRuntimeInitialized = async _ => {

  // create an instance of the encoder and decoder
  const decoder = new openjphjs.HTJ2KDecoder();
  const encoder = new openjphjs.HTJ2KEncoder();
  
  function decode(encodedImagePath, iterations = 1, silent = false) {

    // read encoded bits and copy it into WASM memory
    const encodedBitStream = fs.readFileSync(encodedImagePath);
    const encodedBuffer = decoder.getEncodedBuffer(encodedBitStream.length);
    encodedBuffer.set(encodedBitStream);
  
    // do the actual benchmark
    const beginDecode = process.hrtime();
    for(var i=0; i < iterations; i++) {
      decoder.decode();
    }

    // Get the results
    const frameInfo = decoder.getFrameInfo()
    const decodeDuration = process.hrtime(beginDecode); // hrtime returns seconds/nanoseconds tuple
    const decodeDurationInSeconds = (decodeDuration[0] + (decodeDuration[1] / 1000000000));
    const timePerFrameMS = ((decodeDurationInSeconds / iterations * 1000)) 
    const pixels = (frameInfo.width * frameInfo.height);
    const megaPixels = pixels / (1024.0 * 1024.0);
    const fps = 1000 / timePerFrameMS;
    const mps = (megaPixels)*fps;
  
    // Print out information about the decode
    if(!silent) {
      console.log(`WASM decode ${encodedImagePath} TotalTime: ${decodeDurationInSeconds.toFixed(3)} s for ${iterations} iterations; TPF=${timePerFrameMS.toFixed(3)} ms (${mps.toFixed(2)} MP/s, ${fps.toFixed(2)} FPS)`)
    }
  }
  
  function encode(pathToUncompressedImageFrame, frameInfo, pathToJ2CFile, iterations = 1, silent=false) {
    // Read file and store into WASM Memory
    const uncompressedImageFrame = fs.readFileSync(pathToUncompressedImageFrame);
    const decodedBytes = encoder.getDecodedBuffer(frameInfo);
    decodedBytes.set(uncompressedImageFrame);
    //encoder.setQuality(false, 0.001);
    
    // do the actual benchmark
    const encodeBegin = process.hrtime();
    for(var i=0; i < iterations;i++) {
      encoder.encode();
    }

    // get the results
    const encodeDuration = process.hrtime(encodeBegin);
    const encodeDurationInSeconds = (encodeDuration[0] + (encodeDuration[1] / 1000000000));
    const timePerFrameMS = ((encodeDurationInSeconds / iterations * 1000)) 
    const pixels = (frameInfo.width * frameInfo.height);
    const megaPixels = pixels / (1024.0 * 1024.0);
    const fps = 1000 / timePerFrameMS;
    const mps = (megaPixels)*fps;
  
    // print out information about the encode
    if(!silent) {
      console.log(`WASM encode ${pathToUncompressedImageFrame} TotalTime: ${encodeDurationInSeconds.toFixed(3)} s for ${iterations} iterations; TPF=${timePerFrameMS.toFixed(3)} ms (${mps.toFixed(2)} MP/s, ${fps.toFixed(2)} FPS)`)
    }
  
    if(pathToJ2CFile) {
        //fs.writeFileSync(pathToJ2CFile, encodedBytes);
    }
  }

  // warm up decoder and encoder
  decode('../fixtures/j2c/CT2.j2c', 1, true);
  encode('../fixtures/raw/CT1.RAW', {width: 512, height: 512, bitsPerSample: 16, componentCount: 1, isSigned: true}, '../fixtures/j2c/CT1.j2c', 1, true);

  // benchmark
  const iterations = 20
  decode('../fixtures/j2c/CT1.j2c', iterations);
  decode('../fixtures/j2c/MG1.j2c', iterations);
  encode('../fixtures/raw/CT1.RAW', {width: 512, height: 512, bitsPerSample: 16, componentCount: 1, isSigned: true}, '../fixtures/j2c/CT1.j2c', iterations);

  // J2K Color testing
  //decodeFile("test/fixtures/j2k/US1.j2k", 1);


  // benchark encoding

  //encode('../fixtures/raw/CT2.RAW', {width: 512, height: 512, bitsPerSample: 16, componentCount: 1, isSigned: true}, '../fixtures/j2c/CT2.j2c');
  //encode('../fixtures/raw/MG1.RAW', {width: 3064, height: 4774, bitsPerSample: 16, componentCount: 1, isSigned: false}, '../fixtures/j2c/MG1.j2c');
  //encode('../fixtures/raw/MR1.RAW', {width: 512, height: 512, bitsPerSample: 16, componentCount: 1, isSigned: true}, '../fixtures/j2c/MR1.j2c');
  //encode('../fixtures/raw/MR2.RAW', {width: 1024, height: 1024, bitsPerSample: 16, componentCount: 1, isSigned: false}, '../fixtures/j2c/MR2.j2c');
  //encode('../fixtures/raw/MR3.RAW', {width: 512, height: 512, bitsPerSample: 16, componentCount: 1, isSigned: true}, '../fixtures/j2c/MR3.j2c');
  //encode('../fixtures/raw/MR4.RAW', {width: 512, height: 512, bitsPerSample: 16, componentCount: 1, isSigned: false}, '../fixtures/j2c/MR4.j2c');
  //encode('../fixtures/raw/NM1.RAW', {width: 256, height: 1024, bitsPerSample: 16, componentCount: 1, isSigned: true}, '../fixtures/j2c/NM1.j2c');
  //encode('../fixtures/raw/RG1.RAW', {width: 1841, height: 1955, bitsPerSample: 16, componentCount: 1, isSigned: false}, '../fixtures/j2c/RG1.j2c');
  //encode('../fixtures/raw/RG1.RAW', {width: 1841, height: 1955, bitsPerSample: 16, componentCount: 1, isSigned: false}, '../fixtures/j2c/RG1.j2c');
  //encode('../fixtures/raw/RG2.RAW', {width: 1760, height: 2140, bitsPerSample: 16, componentCount: 1, isSigned: false}, '../fixtures/j2c/RG2.j2c');
  //encode('../fixtures/raw/RG3.RAW', {width: 1760, height: 1760, bitsPerSample: 16, componentCount: 1, isSigned: false}, '../fixtures/j2c/RG3.j2c');
  //encode('../fixtures/raw/SC1.RAW', {width: 2048, height: 2487, bitsPerSample: 16, componentCount: 1, isSigned: false}, '../fixtures/j2c/SC1.j2c');
  //encode('../fixtures/raw/XA1.RAW', {width: 1024, height: 1024, bitsPerSample: 16, componentCount: 1, isSigned: false}, '../fixtures/j2c/XA1.j2c');

  }
