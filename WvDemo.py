"""
** WvDemo.java
**
** Sample program for use with WavPack.py
**
** Copyright (c) 2007-2013 Peter McQuillan
**
** All Rights Reserved.
**                       
** Distributed under the BSD Software License (see license.txt)  
**
"""

import sys
import struct
import WavPack

class RiffChunkHeader :
    ckID = [0] * 4
    ckSize = 0
    formType = [0] * 4

class FmtChunkHeader :
    ckID = [0] * 4
    ckSize = 0

class DtChunkHeader :
    ckID = [0] * 4
    ckSize = 0

class WaveHeader :
    FormatTa = 0
    NumChannels = 0
    SampleRate = 0
    BytesPerSecond = 0
    BlockAlign = 0
    BitsPerSample = 0

# Reformat samples from longs in processor's native endian mode to
# little-endian data with (possibly) less than 4 bytes / sample.

def format_samples( bps, src,  samcnt) :
    temp = 0
    counter = 0
    counter2 = 0
    dst = [0] * 4 * WavPack.SAMPLE_BUFFER_SIZE

    if bps == 1 :
        while (samcnt > 0) :
            dst[counter] = (0x00FF & (src[counter] + 128))
            counter = counter + 1
            samcnt = samcnt - 1
            
    elif bps == 2 :
        while (samcnt > 0) :
            temp = src[counter2]
            dst[counter] = temp & 0xff
            counter = counter + 1
            dst[counter] = (temp & 0xff00) >> 8
            counter = counter + 1
            counter2 = counter2 + 1
            samcnt = samcnt - 1
            
    elif bps == 3 :
        while (samcnt > 0) :
            temp = src[counter2]
            dst[counter] = temp & 0xff
            counter = counter + 1
            dst[counter] = (temp & 0xff00) >> 8
            counter = counter + 1
            dst[counter] = (temp & 0xff0000) >> 16
            counter = counter + 1
            counter2 = counter2 + 1
            samcnt = samcnt - 1

    elif bps == 4 :
        while (samcnt > 0) :
            temp = src[counter2]
            dst[counter] = temp & 0xff
            counter = counter + 1
            dst[counter] = (temp & 0xff00) >> 8
            counter = counter + 1
            dst[counter] = (temp & 0xff0000) >> 16
            counter = counter + 1
            dst[counter] = (temp & 0xff000000) >> 24
            counter = counter + 1
            counter2 = counter2 + 1
            samcnt = samcnt - 1


    return dst



# Start of main routine

temp_buffer = [0] * WavPack.SAMPLE_BUFFER_SIZE
pcm_buffer =  [0] * 4 * WavPack.SAMPLE_BUFFER_SIZE

FormatChunkHeader = FmtChunkHeader()
DataChunkHeader = DtChunkHeader()
myRiffChunkHeader = RiffChunkHeader()
WaveHeader = WaveHeader()
myRiffChunkHeaderAsByteArray = [0] * 12
myFormatChunkHeaderAsByteArray = [0] * 8
myWaveHeaderAsByteArray = [0] * 16
myDataChunkHeaderAsByteArray = [0] * 8

total_unpacked_samples = 0
total_samples = 0
num_channels = 0
bps = 0
start = 0
end = 0

if (len(sys.argv) == 1):
    inputWVFile = "input.wv"
else:
    inputWVFile = sys.argv[1]


try:
    fistream = open(inputWVFile,"rb")
except IOError:
    print "Input file not found"
    exit(1)


wpc = WavPack.WavpackOpenFileInput(fistream)

if (wpc.error) :
    print "Sorry an error has occured"
    print wpc.error_message
    fistream.close()
    exit(1)


num_channels = WavPack.WavpackGetReducedChannels(wpc)

print "The wavpack file has " + str(num_channels) + " channels"

total_samples = WavPack.WavpackGetNumSamples(wpc)

print "The wavpack file has " + str(total_samples )+ " samples"
 
bps = WavPack.WavpackGetBytesPerSample(wpc)

print "The wavpack file has " + str(bps) + " bytes per sample"

myRiffChunkHeader.ckID[0] = ord('R')
myRiffChunkHeader.ckID[1] = ord('I')
myRiffChunkHeader.ckID[2] = ord('F')
myRiffChunkHeader.ckID[3] = ord('F')


myRiffChunkHeader.ckSize = total_samples * num_channels * bps + 8 * 2 + 16 + 4
myRiffChunkHeader.formType[0] = ord('W')
myRiffChunkHeader.formType[1] = ord('A')
myRiffChunkHeader.formType[2] = ord('V')
myRiffChunkHeader.formType[3] = ord('E')

FormatChunkHeader.ckID[0] = ord('f')
FormatChunkHeader.ckID[1] = ord('m')
FormatChunkHeader.ckID[2] = ord('t')
FormatChunkHeader.ckID[3] = ord(' ')

FormatChunkHeader.ckSize = 16;

WaveHeader.FormatTag = 1;
WaveHeader.NumChannels = num_channels;
WaveHeader.SampleRate = WavPack.WavpackGetSampleRate(wpc);
WaveHeader.BlockAlign = num_channels * bps;
WaveHeader.BytesPerSecond = WaveHeader.SampleRate * WaveHeader.BlockAlign;
WaveHeader.BitsPerSample = WavPack.WavpackGetBitsPerSample(wpc);

DataChunkHeader.ckID[0] = ord('d')
DataChunkHeader.ckID[1] = ord('a')
DataChunkHeader.ckID[2] = ord('t')
DataChunkHeader.ckID[3] = ord('a')
DataChunkHeader.ckSize = total_samples * num_channels * bps;


myRiffChunkHeaderAsByteArray[0] = myRiffChunkHeader.ckID[0];
myRiffChunkHeaderAsByteArray[1] = myRiffChunkHeader.ckID[1];
myRiffChunkHeaderAsByteArray[2] = myRiffChunkHeader.ckID[2];
myRiffChunkHeaderAsByteArray[3] = myRiffChunkHeader.ckID[3];

# swap endians here

myRiffChunkHeaderAsByteArray[7] = (myRiffChunkHeader.ckSize >> 24) & 0xff
myRiffChunkHeaderAsByteArray[6] = (myRiffChunkHeader.ckSize >> 16) & 0xff
myRiffChunkHeaderAsByteArray[5] = (myRiffChunkHeader.ckSize >> 8) & 0xff
myRiffChunkHeaderAsByteArray[4] = (myRiffChunkHeader.ckSize) & 0xff

myRiffChunkHeaderAsByteArray[8] = (myRiffChunkHeader.formType[0])
myRiffChunkHeaderAsByteArray[9] = (myRiffChunkHeader.formType[1])
myRiffChunkHeaderAsByteArray[10] = (myRiffChunkHeader.formType[2])
myRiffChunkHeaderAsByteArray[11] = (myRiffChunkHeader.formType[3])

myFormatChunkHeaderAsByteArray[0] = FormatChunkHeader.ckID[0]
myFormatChunkHeaderAsByteArray[1] = FormatChunkHeader.ckID[1]
myFormatChunkHeaderAsByteArray[2] = FormatChunkHeader.ckID[2]
myFormatChunkHeaderAsByteArray[3] = FormatChunkHeader.ckID[3]

# swap endians here
myFormatChunkHeaderAsByteArray[7] = (FormatChunkHeader.ckSize >> 24) & 0xff
myFormatChunkHeaderAsByteArray[6] = (FormatChunkHeader.ckSize >> 16) & 0xff
myFormatChunkHeaderAsByteArray[5] = (FormatChunkHeader.ckSize >> 8) & 0xff
myFormatChunkHeaderAsByteArray[4] = (FormatChunkHeader.ckSize) & 0xff


# swap endians
myWaveHeaderAsByteArray[1] = (WaveHeader.FormatTag >> 8) & 0xff
myWaveHeaderAsByteArray[0] = (WaveHeader.FormatTag) & 0xff

# swap endians
myWaveHeaderAsByteArray[3] = (WaveHeader.NumChannels >> 8) & 0xff
myWaveHeaderAsByteArray[2] = WaveHeader.NumChannels & 0xff


# swap endians
myWaveHeaderAsByteArray[7] = (WaveHeader.SampleRate >> 24) & 0xff
myWaveHeaderAsByteArray[6] = (WaveHeader.SampleRate >> 16) & 0xff
myWaveHeaderAsByteArray[5] = (WaveHeader.SampleRate >> 8) & 0xff
myWaveHeaderAsByteArray[4] = (WaveHeader.SampleRate) & 0xff

# swap endians

myWaveHeaderAsByteArray[11] = (WaveHeader.BytesPerSecond >> 24) & 0xff
myWaveHeaderAsByteArray[10] = (WaveHeader.BytesPerSecond >> 16) & 0xff
myWaveHeaderAsByteArray[9] = (WaveHeader.BytesPerSecond >> 8) & 0xff
myWaveHeaderAsByteArray[8] = (WaveHeader.BytesPerSecond) & 0xff

# swap endians
myWaveHeaderAsByteArray[13] = (WaveHeader.BlockAlign >> 8) & 0xff
myWaveHeaderAsByteArray[12] = WaveHeader.BlockAlign & 0xff

# swap endians
myWaveHeaderAsByteArray[15] = (WaveHeader.BitsPerSample >> 8) & 0xff
myWaveHeaderAsByteArray[14] = WaveHeader.BitsPerSample & 0xff

myDataChunkHeaderAsByteArray[0] = DataChunkHeader.ckID[0];
myDataChunkHeaderAsByteArray[1] = DataChunkHeader.ckID[1];
myDataChunkHeaderAsByteArray[2] = DataChunkHeader.ckID[2];
myDataChunkHeaderAsByteArray[3] = DataChunkHeader.ckID[3];

# swap endians

myDataChunkHeaderAsByteArray[7] = (DataChunkHeader.ckSize >> 24) & 0xff
myDataChunkHeaderAsByteArray[6] = (DataChunkHeader.ckSize >> 16) & 0xff
myDataChunkHeaderAsByteArray[5] = (DataChunkHeader.ckSize >> 8) & 0xff
myDataChunkHeaderAsByteArray[4] = DataChunkHeader.ckSize & 0xff


try :
    fostream = open("output.wav","wb")

    format = "B"
    for i in range(0,12) :
        data = struct.pack(format,myRiffChunkHeaderAsByteArray[i])
        fostream.write(data)

    for i in range(0,8) :
        data = struct.pack(format,myFormatChunkHeaderAsByteArray[i])
        fostream.write(data)

    for i in range(0,16) :
        data = struct.pack(format,myWaveHeaderAsByteArray[i])
        fostream.write(data)

    for i in range(0,8) :
        data = struct.pack(format,myDataChunkHeaderAsByteArray[i])
        fostream.write(data)
        

    newday = 0
    #while newday < 300 :
    while (WavPack.TRUE) :
        # print "ITERATION " + str(newday)
        newday = newday + 1
        samples_unpacked = 0

        samples_unpacked = WavPack.WavpackUnpackSamples(wpc, temp_buffer, WavPack.SAMPLE_BUFFER_SIZE / num_channels);

        total_unpacked_samples += samples_unpacked

        if (samples_unpacked > 0) :
            samples_unpacked = samples_unpacked * num_channels

            pcm_buffer = format_samples(bps, temp_buffer, samples_unpacked)

            binarydata = ""
            for i in range(0, samples_unpacked * bps) :
                binarydata += chr(pcm_buffer[i])
                
            fostream.write(binarydata)
                

        if (samples_unpacked == 0) :
            break

except IOError:
    print "Error when writing wav file, sorry: "
    fistream.close()
    fostream.close()
    exit(1)
except :
    print "General error when writing wav file, sorry: "
    fistream.close()
    fostream.close()
    exit(1)  




if ((WavPack.WavpackGetNumSamples(wpc) != -1)
    and (total_unpacked_samples != WavPack.WavpackGetNumSamples(wpc))) :
    print "Incorrect number of samples" 
    fistream.close()
    fostream.close()
    exit(1)

if (WavPack.WavpackGetNumErrors(wpc) > 0) :
    print "CRC errors detected"
    fistream.close()
    fostream.close()
    exit(1)


fistream.close()
fostream.close()
print "Finished!"


