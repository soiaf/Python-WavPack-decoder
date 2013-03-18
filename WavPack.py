"""
** WavPack.py
**
** Implementation of WavPack decoder written in Python
**
** Copyright (c) 2007-2013 Peter McQuillan
**
** All Rights Reserved.
**                       
** Distributed under the BSD Software License (see license.txt)  
**
"""

import sys

# Change the following value to an even number to reflect the maximum number of samples to be processed
# per call to WavpackUnpackSamples()

SAMPLE_BUFFER_SIZE = 256
FALSE = 0
TRUE = 1

BYTES_STORED = 3;       # 1-4 bytes/sample
MONO_FLAG  = 4;       # not stereo
HYBRID_FLAG = 8;       # hybrid mode
FALSE_STEREO = 0x40000000;      # block is stereo, but data is mono

SHIFT_LSB = 13;
SHIFT_MASK = (0x1fL << SHIFT_LSB);

FLOAT_DATA  = 0x80;    # ieee 32-bit floating point data

SRATE_LSB = 23;
SRATE_MASK = (0xfL << SRATE_LSB);

FINAL_BLOCK = 0x1000;  # final block of multichannel segment

MIN_STREAM_VERS = 0x402;       # lowest stream version we'll decode
MAX_STREAM_VERS = 0x410;       # highest stream version we'll decode


ID_DUMMY            =    0x0;
ID_ENCODER_INFO     =    0x1;
ID_DECORR_TERMS     =    0x2;
ID_DECORR_WEIGHTS   =    0x3;
ID_DECORR_SAMPLES   =    0x4;
ID_ENTROPY_VARS     =    0x5;
ID_HYBRID_PROFILE   =    0x6;
ID_SHAPING_WEIGHTS  =    0x7;
ID_FLOAT_INFO       =    0x8;
ID_INT32_INFO       =    0x9;
ID_WV_BITSTREAM     =    0xa;
ID_WVC_BITSTREAM    =    0xb;
ID_WVX_BITSTREAM    =    0xc;
ID_CHANNEL_INFO     =    0xd;

JOINT_STEREO  =  0x10;    # joint stereo
CROSS_DECORR  =  0x20;    # no-delay cross decorrelation
HYBRID_SHAPE  =  0x40;    # noise shape (hybrid mode only)

INT32_DATA     = 0x100;   # special extended int handling
HYBRID_BITRATE = 0x200;   # bitrate noise (hybrid mode only)
HYBRID_BALANCE = 0x400;   # balance noise (hybrid stereo mode only)

INITIAL_BLOCK  = 0x800;   # initial block of multichannel segment

FLOAT_SHIFT_ONES = 1;      # bits left-shifted into float = '1'
FLOAT_SHIFT_SAME = 2;      # bits left-shifted into float are the same
FLOAT_SHIFT_SENT = 4;      # bits shifted into float are sent literally
FLOAT_ZEROS_SENT = 8;      # "zeros" are not all real zeros
FLOAT_NEG_ZEROS  = 0x10;   # contains negative zeros
FLOAT_EXCEPTIONS = 0x20;   # contains exceptions (inf, nan, etc.)


ID_OPTIONAL_DATA      =  0x20;
ID_ODD_SIZE           =  0x40;
ID_LARGE              =  0x80;

MAX_NTERMS = 16;
MAX_TERM = 8;

MAG_LSB = 18;
MAG_MASK = (0x1fL << MAG_LSB);

ID_RIFF_HEADER   = 0x21;
ID_RIFF_TRAILER  = 0x22;
ID_REPLAY_GAIN   = 0x23;
ID_CUESHEET      = 0x24;
ID_CONFIG_BLOCK    = 0x25;
ID_MD5_CHECKSUM  = 0x26;
ID_SAMPLE_RATE   = 0x27;

CONFIG_BYTES_STORED    = 3;       # 1-4 bytes/sample
CONFIG_MONO_FLAG       = 4;       # not stereo
CONFIG_HYBRID_FLAG     = 8;       # hybrid mode
CONFIG_JOINT_STEREO    = 0x10;    # joint stereo
CONFIG_CROSS_DECORR    = 0x20;    # no-delay cross decorrelation
CONFIG_HYBRID_SHAPE    = 0x40;    # noise shape (hybrid mode only)
CONFIG_FLOAT_DATA      = 0x80;    # ieee 32-bit floating point data
CONFIG_FAST_FLAG       = 0x200;   # fast mode
CONFIG_HIGH_FLAG       = 0x800;   # high quality mode
CONFIG_VERY_HIGH_FLAG  = 0x1000;  # very high
CONFIG_BITRATE_KBPS    = 0x2000;  # bitrate is kbps, not bits / sample
CONFIG_AUTO_SHAPING    = 0x4000;  # automatic noise shaping
CONFIG_SHAPE_OVERRIDE  = 0x8000;  # shaping mode specified
CONFIG_JOINT_OVERRIDE  = 0x10000; # joint-stereo mode specified
CONFIG_CREATE_EXE      = 0x40000; # create executable
CONFIG_CREATE_WVC      = 0x80000; # create correction file
CONFIG_OPTIMIZE_WVC    = 0x100000; # maximize bybrid compression
CONFIG_CALC_NOISE      = 0x800000; # calc noise in hybrid mode
CONFIG_LOSSY_MODE      = 0x1000000; # obsolete (for information)
CONFIG_EXTRA_MODE      = 0x2000000; # extra processing mode
ONFIG_SKIP_WVX        = 0x4000000; # no wvx stream w/ floats & big ints
CONFIG_MD5_CHECKSUM    = 0x8000000; # compute & store MD5 signature
CONFIG_OPTIMIZE_MONO   = 0x80000000; # optimize for mono streams posing as stereo

MODE_WVC        = 0x1;
MODE_LOSSLESS   = 0x2;
MODE_HYBRID     = 0x4;
MODE_FLOAT      = 0x8;
MODE_VALID_TAG  = 0x10;
MODE_HIGH       = 0x20;
MODE_FAST       = 0x40;


sample_rates = (6000, 8000, 9600, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000, 64000, 88200, 96000, 192000)


LIMIT_ONES = 16  # maximum consecutive 1s sent for "div" data

# these control the time constant "slow_level" which is used for hybrid mode
# that controls bitrate as a function of residual level (HYBRID_BITRATE).
SLS = 8
SLO = ((1 << (SLS - 1)))


# these control the time constant of the 3 median level breakpoints
DIV0 = 128   # 5/7 of samples
DIV1 = 64    # 10/49 of samples
DIV2 = 32    # 20/343 of samples



nbits_table = (
0, 1, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4,      # 0 - 15
5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,      # 16 - 31
6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,      # 32 - 47
6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,      # 48 - 63
7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,      # 64 - 79
7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,      # 80 - 95
7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,      # 96 - 111
7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,      # 112 - 127
8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,      # 128 - 143
8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,      # 144 - 159
8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,      # 160 - 175
8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,      # 176 - 191
8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,      # 192 - 207
8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,      # 208 - 223
8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,      # 224 - 239
8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8)      # 240 - 255



log2_table = (
0x00, 0x01, 0x03, 0x04, 0x06, 0x07, 0x09, 0x0a, 0x0b, 0x0d, 0x0e, 0x10, 0x11, 0x12, 0x14, 0x15,
0x16, 0x18, 0x19, 0x1a, 0x1c, 0x1d, 0x1e, 0x20, 0x21, 0x22, 0x24, 0x25, 0x26, 0x28, 0x29, 0x2a,
0x2c, 0x2d, 0x2e, 0x2f, 0x31, 0x32, 0x33, 0x34, 0x36, 0x37, 0x38, 0x39, 0x3b, 0x3c, 0x3d, 0x3e,
0x3f, 0x41, 0x42, 0x43, 0x44, 0x45, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4d, 0x4e, 0x4f, 0x50, 0x51,
0x52, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63,
0x64, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71, 0x72, 0x74, 0x75,
0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81, 0x82, 0x83, 0x84, 0x85,
0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91, 0x92, 0x93, 0x94, 0x95,
0x96, 0x97, 0x98, 0x99, 0x9a, 0x9b, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1, 0xa2, 0xa3, 0xa4,
0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb0, 0xb1, 0xb2, 0xb2,
0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf, 0xc0, 0xc0,
0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xcb, 0xcb, 0xcc, 0xcd, 0xce,
0xcf, 0xd0, 0xd0, 0xd1, 0xd2, 0xd3, 0xd4, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd8, 0xd9, 0xda, 0xdb,
0xdc, 0xdc, 0xdd, 0xde, 0xdf, 0xe0, 0xe0, 0xe1, 0xe2, 0xe3, 0xe4, 0xe4, 0xe5, 0xe6, 0xe7, 0xe7,
0xe8, 0xe9, 0xea, 0xea, 0xeb, 0xec, 0xed, 0xee, 0xee, 0xef, 0xf0, 0xf1, 0xf1, 0xf2, 0xf3, 0xf4,
0xf4, 0xf5, 0xf6, 0xf7, 0xf7, 0xf8, 0xf9, 0xf9, 0xfa, 0xfb, 0xfc, 0xfc, 0xfd, 0xfe, 0xff, 0xff)


exp2_table = (
0x00, 0x01, 0x01, 0x02, 0x03, 0x03, 0x04, 0x05, 0x06, 0x06, 0x07, 0x08, 0x08, 0x09, 0x0a, 0x0b,
0x0b, 0x0c, 0x0d, 0x0e, 0x0e, 0x0f, 0x10, 0x10, 0x11, 0x12, 0x13, 0x13, 0x14, 0x15, 0x16, 0x16,
0x17, 0x18, 0x19, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1d, 0x1e, 0x1f, 0x20, 0x20, 0x21, 0x22, 0x23,
0x24, 0x24, 0x25, 0x26, 0x27, 0x28, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2c, 0x2d, 0x2e, 0x2f, 0x30,
0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3a, 0x3b, 0x3c, 0x3d,
0x3e, 0x3f, 0x40, 0x41, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x48, 0x49, 0x4a, 0x4b,
0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a,
0x5b, 0x5c, 0x5d, 0x5e, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x87, 0x88, 0x89, 0x8a,
0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91, 0x92, 0x93, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0x9b,
0x9c, 0x9d, 0x9f, 0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad,
0xaf, 0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xbc, 0xbd, 0xbe, 0xbf, 0xc0,
0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc8, 0xc9, 0xca, 0xcb, 0xcd, 0xce, 0xcf, 0xd0, 0xd2, 0xd3, 0xd4,
0xd6, 0xd7, 0xd8, 0xd9, 0xdb, 0xdc, 0xdd, 0xde, 0xe0, 0xe1, 0xe2, 0xe4, 0xe5, 0xe6, 0xe8, 0xe9,
0xea, 0xec, 0xed, 0xee, 0xf0, 0xf1, 0xf2, 0xf4, 0xf5, 0xf6, 0xf8, 0xf9, 0xfa, 0xfc, 0xfd, 0xff )



ones_count_table = (
0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,4,0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,5,
0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,4,0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,6,
0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,4,0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,5,
0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,4,0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,7,
0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,4,0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,5,
0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,4,0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,6,
0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,4,0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,5,
0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,4,0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,8)


class entropy_data :
    def __init__(self):
        self.slow_level = 0
        self.median = [0] * 3
        self.error_limit = 0

class words_data :
    def __init__(self):
        self.bitrate_delta = [0] * 2
        self.bitrate_acc = [0] * 2
        self.pend_data = 0
        self.holding_one = 0
        self.zeros_acc = 0
        self.holding_zero = 0
        self.pend_count = 0
        self.temp_ed1 = entropy_data()
        self.temp_ed2 = entropy_data()
        self.c = [ self.temp_ed1 , self.temp_ed2 ]
         
class decorr_pass:
    def __init__(self):
        self.term = 0
        self.delta = 0
        self.weight_A = 0
        self.weight_B = 0
        self.samples_A = [0] * MAX_TERM
        self.samples_B = [0] * MAX_TERM

class WavpackHeader :
    ckID = [0] * 4
    ckSize = 0    
    version = 0
    track_no = 0
    index_no = 0    
    total_samples = 0
    block_index = 0
    block_samples = 0
    flags = 0
    crc = 0    
    status = 0;    # 1 means error

class WavpackMetadata :
    def __init__(self):
        self.byte_length = 0
        self.data = [0] * 1024
        self.id = 0
        self.hasdata = 0;    # 0 does not have data, 1 has data
        self.status = 0;    # 0 ok, 1 error

class WavpackConfig :
    bits_per_sample = 0
    bytes_per_sample = 0
    num_channels = 0
    norm_exp = 0
    flags = 0
    sample_rate = 0
    channel_mask = 0

class Bitstream :
    def __init__(self):
        self.end = 0
        self.ptr = 0
        self.file_bytes = 0
        self.sr = 0
        self.error = 0
        self.bc = 0
        self.file = None;
        self.bitval = 0;
        self.buf = [0] * 1024
        self.buf_index = 0

class WavpackStream:
    def __init__(self):
        self.wphdr = WavpackHeader()
        self.wvbits = Bitstream()
        self.w = words_data()

        num_terms = 0
        mute_error = 0
        sample_index = 0
        crc = 0    

        int32_sent_bits = 0
        int32_zeros = 0
        int32_ones = 0
        int32_dups = 0  
        float_flags = 0
        float_shift = 0
        float_max_exp = 0
        self.float_norm_exp = 0
     
        dp1 =  decorr_pass();
        dp2 =  decorr_pass();
        dp3 =  decorr_pass();
        dp4 =  decorr_pass();
        dp5 =  decorr_pass();
        dp6 =  decorr_pass();
        dp7 =  decorr_pass();
        dp8 =  decorr_pass();
        dp9 =  decorr_pass();
        dp10 =  decorr_pass();
        dp11 =  decorr_pass();
        dp12 =  decorr_pass();
        dp13 =  decorr_pass();
        dp14 =  decorr_pass();
        dp15 =  decorr_pass();
        dp16 =  decorr_pass();

        self.decorr_passes =  [dp1, dp2, dp3, dp4, dp5, dp6, dp7, dp8, dp9, dp10, dp11, dp12, dp13, dp14, dp15, dp16]

class WavpackContext:
    def __init__(self):
        self.config = WavpackConfig()
        self.stream = WavpackStream()
        self.READ_BUFFER_SIZE = 1024

        self.read_buffer = [0] * self.READ_BUFFER_SIZE
        self.error_message = ""
        self.error = FALSE
        self.infile = 0
        self.total_samples = 0
        self.crc_errors = 0
        self.first_flags = 0        
        self.open_flags = 0
        self.norm_offset = 0
        self.reduced_channels = 0
        self.lossy_blocks = 0
        self.status = 0;    # 0 ok, 1 error

class case_selector(Exception):
   def __init__(self, value): # overridden to ensure we've got a value argument
      Exception.__init__(self, value)

def switch(variable):
   raise case_selector(variable)

def case(value):
   exclass, exobj, tb = sys.exc_info()
   if exclass is case_selector and exobj.args[0] == value: return exclass
   return None

def multicase(*values):
   exclass, exobj, tb = sys.exc_info()
   if exclass is case_selector and exobj.args[0] in values: return exclass
   return None


# This function reads data from the specified stream in search of a valid
# WavPack 4.0 audio block. If this fails in 1 megabyte (or an invalid or
# unsupported WavPack block is encountered) then an appropriate message is
# copied to "error" and NULL is returned, otherwise a pointer to a
# WavpackContext structure is returned (which is used to call all other
# functions in this module). This can be initiated at the beginning of a
# WavPack file, or anywhere inside a WavPack file. To determine the exact
# position within the file use WavpackGetSampleIndex().  Also,
# this function will not handle "correction" files, plays only the first
# two channels of multi-channel files, and is limited in resolution in some
# large integer or floating point files (but always provides at least 24 bits
# of resolution).

def WavpackOpenFileInput(infile):
    wpc = WavpackContext();
    wps = wpc.stream;

    wpc.infile = infile;
    wpc.total_samples = -1;
    wpc.norm_offset = 0;
    wpc.open_flags = 0;


    # open the source file for reading and store the size

    while (wps.wphdr.block_samples == 0) :
        wps.wphdr = read_next_header(wpc.infile, wps.wphdr);

        if (wps.wphdr.status == 1) :
            wpc.error_message = "not compatible with this version of WavPack file!";
            wpc.error = TRUE;
            return (wpc);

        if (wps.wphdr.block_samples > 0 and wps.wphdr.total_samples != -1) :
            wpc.total_samples = wps.wphdr.total_samples;

        # lets put the stream back in the context

        wpc.stream = wps;

        if ((unpack_init(wpc)) == FALSE) :
            wpc.error = TRUE;
            return wpc;

    wpc.config.flags = wpc.config.flags & ~0xff;
    wpc.config.flags = wpc.config.flags | (wps.wphdr.flags & 0xff);

    wpc.config.bytes_per_sample = ((wps.wphdr.flags & BYTES_STORED) + 1);
    wpc.config.float_norm_exp = wps.float_norm_exp;

    wpc.config.bits_per_sample = ((wpc.config.bytes_per_sample * 8) \
        - ((wps.wphdr.flags & SHIFT_MASK) >> SHIFT_LSB));

    if ((wpc.config.flags & FLOAT_DATA) > 0) :
        wpc.config.bytes_per_sample = 3
        wpc.config.bits_per_sample = 24

    if (wpc.config.sample_rate == 0) :
        if (wps.wphdr.block_samples == 0 or (wps.wphdr.flags & SRATE_MASK) == SRATE_MASK) :
            wpc.config.sample_rate = 44100
        else :
            wpc.config.sample_rate = sample_rates[((wps.wphdr.flags & SRATE_MASK) \
                >> SRATE_LSB)]

    if (wpc.config.num_channels == 0) :
        if ((wps.wphdr.flags & MONO_FLAG) > 0) :
            wpc.config.num_channels = 1
        else :
            wpc.config.num_channels = 2

        wpc.config.channel_mask = 0x5 - wpc.config.num_channels

    if ((wps.wphdr.flags & FINAL_BLOCK) == 0) :
        if ((wps.wphdr.flags & MONO_FLAG) != 0) :
            wpc.reduced_channels = 1
        else :
            wpc.reduced_channels = 2

    return wpc


# This function obtains general information about an open file and returns
# a mask with the following bit values:

# MODE_LOSSLESS:  file is lossless (pure lossless only)
# MODE_HYBRID:  file is hybrid mode (lossy part only)
# MODE_FLOAT:  audio data is 32-bit ieee floating point (but will provided
#               in 24-bit integers for convenience)
# MODE_HIGH:  file was created in "high" mode (information only)
# MODE_FAST:  file was created in "fast" mode (information only)


def WavpackGetMode (wpc) :
    
    mode = 0;

    if (None != wpc) : 
        if ( (wpc.config.flags & CONFIG_HYBRID_FLAG) != 0) :
            mode |= MODE_HYBRID
        elif ((wpc.config.flags & CONFIG_LOSSY_MODE)==0) :
            mode |= MODE_LOSSLESS

        if (wpc.lossy_blocks != 0) :
            mode &= ~MODE_LOSSLESS

        if ( (wpc.config.flags & CONFIG_FLOAT_DATA) != 0) :
            mode |= MODE_FLOAT

        if ( (wpc.config.flags & CONFIG_HIGH_FLAG) != 0) :
            mode |= MODE_HIGH

        if ( (wpc.config.flags & CONFIG_FAST_FLAG) != 0) :
            mode |= MODE_FAST

    return mode



# Unpack the specified number of samples from the current file position.
# Note that "samples" here refers to "complete" samples, which would be
# 2 longs for stereo files. The audio data is returned right-justified in
# 32-bit longs in the endian mode native to the executing processor. So,
# if the original data was 16-bit, then the values returned would be
# +/-32k. Floating point data will be returned as 24-bit integers (and may
# also be clipped). The actual number of samples unpacked is returned,
# which should be equal to the number requested unless the end of fle is
# encountered or an error occurs.

def  WavpackUnpackSamples(wpc, buffer, samples) :
    wps = wpc.stream;
    samples_unpacked = 0
    samples_to_unpack = 0
    num_channels = wpc.config.num_channels
    bcounter = 0

    temp_buffer = [0] * SAMPLE_BUFFER_SIZE
    buf_idx = 0
    bytes_returned = 0

    while (samples > 0) :
        if (wps.wphdr.block_samples == 0 or (wps.wphdr.flags & INITIAL_BLOCK) == 0
            or wps.sample_index >= wps.wphdr.block_index
            + wps.wphdr.block_samples) :

            wps.wphdr = read_next_header(wpc.infile, wps.wphdr)

            if (wps.wphdr.status == 1) :
                break;

            if (wps.wphdr.block_samples == 0 or wps.sample_index == wps.wphdr.block_index) :
                if ((unpack_init(wpc)) == FALSE) :
                    break;

        if (wps.wphdr.block_samples == 0 or (wps.wphdr.flags & INITIAL_BLOCK) == 0
            or wps.sample_index >= wps.wphdr.block_index
            + wps.wphdr.block_samples) :
            continue;

        if (wps.sample_index < wps.wphdr.block_index) :
            samples_to_unpack = wps.wphdr.block_index - wps.sample_index;

            if (samples_to_unpack > samples) :
                samples_to_unpack = samples

            wps.sample_index += samples_to_unpack;
            samples_unpacked += samples_to_unpack;
            samples -= samples_to_unpack;

            if (wpc.reduced_channels > 0) :
                samples_to_unpack *= wpc.reduced_channels;
            else :
                samples_to_unpack *= num_channels;

            while (samples_to_unpack > 0) :
                temp_buffer[bcounter] = 0
                bcounter = bcounter + 1
                samples_to_unpack = samples_to_unpack - 1

            continue

        samples_to_unpack = wps.wphdr.block_index + wps.wphdr.block_samples - wps.sample_index

        if (samples_to_unpack > samples) :
            samples_to_unpack = samples

        for mycleanup in range(0,256) :
            temp_buffer[mycleanup] = 0
        
        unpack_samples(wpc, temp_buffer, samples_to_unpack)

        if (wpc.reduced_channels > 0) :
            bytes_returned = (samples_to_unpack * wpc.reduced_channels)
        else :
            bytes_returned = (samples_to_unpack * num_channels)

        tempcount = 0
        for mycount in range (buf_idx, buf_idx+bytes_returned) :
            buffer[mycount] = temp_buffer[tempcount]
            tempcount = tempcount + 1

        buf_idx += bytes_returned;

        samples_unpacked += samples_to_unpack;
        samples -= samples_to_unpack;

        if (wps.sample_index == wps.wphdr.block_index + wps.wphdr.block_samples) :
            if (check_crc_error(wpc) > 0) :
                wpc.crc_errors = wpc.crc_errors + 1

        if (wps.sample_index == wpc.total_samples) :
            break;

    return (samples_unpacked)


# Get total number of samples contained in the WavPack file, or -1 if unknown

def WavpackGetNumSamples(wpc) :
    # -1 would mean an unknown number of samples

    if( None != wpc) :
        return (wpc.total_samples)
    else :
        return -1


# Get the current sample index position, or -1 if unknown

def WavpackGetSampleIndex (wpc) :
    if (null != wpc) :
        return wpc.stream.sample_index;

    return -1;


# Get the number of errors encountered so far

def WavpackGetNumErrors(wpc) :
    if( None != wpc) :
        return wpc.crc_errors
    else :
        return 0


# return if any uncorrected lossy blocks were actually written or read

def WavpackLossyBlocks (wpc) :
    if(None != wpc) :
         return wpc.lossy_blocks
    else :
        return 0


# Returns the sample rate of the specified WavPack file

def WavpackGetSampleRate(wpc) :
    if ( None != wpc and wpc.config.sample_rate != 0) :
        return wpc.config.sample_rate
    else :
        return 44100

        
# Returns the number of channels of the specified WavPack file. Note that
# this is the actual number of channels contained in the file, but this
# version can only decode the first two.

def WavpackGetNumChannels(wpc) :
    if ( None != wpc and wpc.config.num_channels != 0) :
        return wpc.config.num_channels
    else :
        return 2


# Returns the actual number of valid bits per sample contained in the
# original file, which may or may not be a multiple of 8. Floating data
# always has 32 bits, integers may be from 1 to 32 bits each. When this
# value is not a multiple of 8, then the "extra" bits are located in the
# LSBs of the results. That is, values are right justified when unpacked
# into longs, but are left justified in the number of bytes used by the
# original data.

def WavpackGetBitsPerSample(wpc) :
    if (None != wpc and wpc.config.bits_per_sample != 0) :
        return wpc.config.bits_per_sample
    else :
        return 16


# Returns the number of bytes used for each sample (1 to 4) in the original
# file. This is required information for the user of this module because the
# audio data is returned in the LOWER bytes of the long buffer and must be
# left-shifted 8, 16, or 24 bits if normalized longs are required.

def WavpackGetBytesPerSample(wpc) :
    if ( None != wpc and wpc.config.bytes_per_sample != 0) :
        return wpc.config.bytes_per_sample
    else :
        return 2


# This function will return the actual number of channels decoded from the
# file (which may or may not be less than the actual number of channels, but
# will always be 1 or 2). Normally, this will be the front left and right
# channels of a multi-channel file.

def WavpackGetReducedChannels(wpc) :
    if (None != wpc and wpc.reduced_channels != 0) :
        return wpc.reduced_channels
    elif (None != wpc and wpc.config.num_channels != 0) :
        return wpc.config.num_channels
    else :
        return 2



# Read from current file position until a valid 32-byte WavPack 4.0 header is
# found and read into the specified pointer. If no WavPack header is found within 1 meg,
# then an error is returned. No additional bytes are read past the header. 

def read_next_header(infile, wphdr) :
    buffer = [0] * 32 # 32 is the size of a WavPack Header
    temp = [0] * 32

    bytes_skipped = 0
    bleft = 0  # bytes left in buffer
    counter = 0
    i = 0

    while (TRUE) :
        for i in range(0,bleft) :
            buffer[i] = buffer[32 - bleft + i];

        counter = 0;

        try :
            temp = infile.read(32 - bleft)
        except:
            wphdr.status = 1;
            return wphdr;

        # Check if we are at the end of the file
        if len(temp) < (32-bleft) :
            wphdr.status = 1;
            return wphdr;            

        for i in range(0, 32 - bleft) : 
            buffer[bleft + i] = temp[i];

        bleft = 32;

        buf4 = ord(buffer[4])
        buf6 = ord(buffer[6])
        buf7 = ord(buffer[7])
        buf8 = ord(buffer[8])
        buf9 = ord(buffer[9])
        
        if  buffer[0] == 'w' and buffer[1] == 'v' and buffer[2] == 'p' and buffer[3] == 'k' \
            and (buf4 & 1) == 0 and buf6 < 16 and buf7 == 0 and buf9 == 4 \
            and buf8 >= (MIN_STREAM_VERS & 0xff) and buf8 <= (MAX_STREAM_VERS & 0xff) :

            wphdr.ckID[0] = 'w';
            wphdr.ckID[1] = 'v';
            wphdr.ckID[2] = 'p';
            wphdr.ckID[3] = 'k';

            wphdr.ckSize =  (ord(buffer[7]) & 0xFF) << 24
            wphdr.ckSize += (ord(buffer[6]) & 0xFF) << 16
            wphdr.ckSize += (ord(buffer[5]) & 0xFF) << 8
            wphdr.ckSize +=  ord(buffer[4]) & 0xFF

            wphdr.version =  ord(buffer[9]) << 8
            wphdr.version += ord(buffer[8])

            wphdr.track_no = ord(buffer[10])
            wphdr.index_no = ord(buffer[11])

            wphdr.total_samples =  (ord(buffer[15]) & 0xFF) << 24
            wphdr.total_samples += (ord(buffer[14]) & 0xFF) << 16
            wphdr.total_samples += (ord(buffer[13]) & 0xFF) << 8
            wphdr.total_samples +=  ord(buffer[12]) & 0xFF

            wphdr.block_index =  (ord(buffer[19]) & 0xFF) << 24
            wphdr.block_index += (ord(buffer[18]) & 0xFF) << 16
            wphdr.block_index += (ord(buffer[17]) & 0xFF) << 8
            wphdr.block_index +=  ord(buffer[16]) & 0XFF

            wphdr.block_samples =  (ord(buffer[23]) & 0xFF) << 24
            wphdr.block_samples += (ord(buffer[22]) & 0xFF) << 16
            wphdr.block_samples += (ord(buffer[21]) & 0xFF) << 8
            wphdr.block_samples +=  ord(buffer[20]) & 0XFF

            wphdr.flags =  (ord(buffer[27]) & 0xFF) << 24
            wphdr.flags += (ord(buffer[26]) & 0xFF) << 16
            wphdr.flags += (ord(buffer[25]) & 0xFF) << 8
            wphdr.flags +=  ord(buffer[24]) & 0xFF

            wphdr.crc =  (ord(buffer[31]) & 0xFF) << 24
            wphdr.crc += (ord(buffer[30]) & 0xFF) << 16
            wphdr.crc += (ord(buffer[29]) & 0xFF) << 8
            wphdr.crc +=  ord(buffer[28]) & 0xFF

            wphdr.status = 0;

            return wphdr;
        else :
            counter = counter + 1
            bleft = bleft - 1

        while (bleft > 0 and buffer[counter] != 'w') :
            counter = counter + 1
            bleft = bleft - 1

        bytes_skipped = bytes_skipped + counter;

        if (bytes_skipped > 1048576L) :
            wphdr.status = 1;
            return wphdr;


def getbit(bs) :
    uns_buf = 0

    if (bs.bc > 0) :
        bs.bc = bs.bc - 1
    else :
        bs.ptr = bs.ptr + 1
        bs.buf_index = bs.buf_index + 1
        bs.bc = 7;

        if (bs.ptr == bs.end) :
            # wrap call here
            bs = bs_read(bs);
        uns_buf = ord(bs.buf[bs.buf_index]) & 0xff
        bs.sr = uns_buf;

    bs.bitval = (bs.sr & 1);
    bs.sr = bs.sr >> 1;

    return bs;


def getbits(nbits, bs) :
    uns_buf = 0
    value = 0

    while ((nbits) > bs.bc) :
        bs.ptr = bs.ptr + 1
        bs.buf_index = bs.buf_index + 1

        if (bs.ptr == bs.end) :
            bs = bs_read(bs);
        uns_buf = ord(bs.buf[bs.buf_index]) & 0xff
        bs.sr = bs.sr | (uns_buf << bs.bc); # values in buffer must be unsigned
        bs.sr = bs.sr & 0xffffffff # bs.sr is unsigned 32 bit
        bs.bc += 8;

    value = bs.sr;

    if (bs.bc > 32) :
        bs.bc -= (nbits);
        bs.sr = (ord(bs.buf[bs.buf_index]) & 0xff) >> (8 - bs.bc);
    else :
        bs.bc -= (nbits);
        bs.sr >>= (nbits);

    return (value)


def bs_open_read(stream, buffer_start, buffer_end, file, file_bytes, passed) :
    bs = Bitstream();

    bs.buf = stream;
    bs.buf_index = buffer_start;
    bs.end = buffer_end;
    bs.sr = 0;
    bs.bc = 0;

    if (passed != 0) :
        bs.ptr = bs.end - 1;
        bs.file_bytes = file_bytes;
        bs.file = file;
    else :
        # Strange to set an index to -1, but the very first call to getbit will iterate this 
        bs.buf_index = -1;
        bs.ptr = -1;

    return bs;


def bs_read(bs) :
    if (bs.file_bytes > 0) :
        bytes_read = 0
        bytes_to_read = 1024;

        if (bytes_to_read > bs.file_bytes) :
            bytes_to_read = bs.file_bytes;

        try :
            buf = [0] * 1024
            buf = bs.file.read(bytes_to_read)
            bytes_read = bytes_to_read
            bs.buf_index = 0
            bs.buf = buf
        except :
            bytes_read = 0;

        if (bytes_read > 0) :
            bs.end = bytes_read
            bs.file_bytes -= bytes_read;
        else :
            for i in range(0, bs.end - bs.buf_index) :
                bs.buf[i] = -1
            bs.error = 1;
    else :
        bs.error = 1;

    if (bs.error > 0) :
        for i in range(0,bs.end - bs.buf_index) :
            bs.buf[i] = -1;

    bs.ptr = 0;
    bs.buf_index = 0;

    return bs;



def read_float_info (wps, wpmd) :
    bytecnt = wpmd.byte_length
    byteptr = wpmd.data
    counter = 0;

    if bytecnt != 4 :
        return FALSE

    wps.float_flags = ord(byteptr[counter])
    counter = counter + 1
    wps.float_shift = ord(byteptr[counter])
    counter = counter + 1
    wps.float_max_exp = ord(byteptr[counter])
    counter = counter + 1
    wps.float_norm_exp = ord(byteptr[counter])

    return TRUE;


def float_values (wps, values, num_values) :
    shift = wps.float_max_exp - wps.float_norm_exp + wps.float_shift
    value_counter = 0

    if (shift > 32) :
        shift = 32
    elif (shift < -32) :
        shift = -32

    while (num_values>0) :
        if (shift > 0) :
            values[value_counter] <<= shift
        elif (shift < 0) :
            values[value_counter] >>= -shift

        if (values[value_counter] > 8388607L) :
            values[value_counter] = 8388607L
        elif (values[value_counter] < -8388608L) :
            values[value_counter] = -8388608L

        value_counter = value_counter + 1
        num_values = num_values - 1

    return values



def read_metadata_buff(wpc, wpmd) :
    bytes_to_read = 0;
    tchar = 0;

    try :
        wpmd.id = ord(wpc.infile.read(1))
        tchar =   ord(wpc.infile.read(1))
    except:
        wpmd.status = 1;
        return FALSE

    wpmd.byte_length = tchar << 1;

    if ((wpmd.id & ID_LARGE) != 0) :
        wpmd.id &= ~ID_LARGE;

        try :
            tchar = ord(wpc.infile.read(1));
        except:
            wpmd.status = 1;
            return FALSE;

        wpmd.byte_length += tchar << 9;

        try :
            tchar = ord(wpc.infile.read(1));
        except:
            wpmd.status = 1;
            return FALSE;

        wpmd.byte_length += tchar << 17;

    if ((wpmd.id & ID_ODD_SIZE) != 0) :
        wpmd.id &= ~ID_ODD_SIZE;
        wpmd.byte_length = wpmd.byte_length - 1

    if (wpmd.byte_length == 0 or wpmd.id == ID_WV_BITSTREAM) :
        wpmd.hasdata = FALSE;
        return TRUE;

    bytes_to_read = wpmd.byte_length + (wpmd.byte_length & 1)
    
    if (bytes_to_read > wpc.READ_BUFFER_SIZE) :
        bytes_read = 0
        wpmd.hasdata = FALSE;

        while (bytes_to_read > wpc.READ_BUFFER_SIZE) :
            try :
                wpc.read_buffer = wpc.infile.read( wpc.READ_BUFFER_SIZE )
                bytes_read = wpc.READ_BUFFER_SIZE
                if(bytes_read != wpc.READ_BUFFER_SIZE) :
                    return FALSE;
            except:
                return FALSE;
            bytes_to_read -= wpc.READ_BUFFER_SIZE
    else :
        wpmd.hasdata = TRUE;
        wpmd.data = wpc.read_buffer;

    if (bytes_to_read != 0) :
        bytes_read = 0

        try :
            wpc.read_buffer = wpc.infile.read(bytes_to_read)
            wpmd.data = wpc.read_buffer
            bytes_read = bytes_to_read
            if(bytes_read !=  bytes_to_read) :
                wpmd.hasdata = FALSE;
                return FALSE;
        except:
            wpmd.hasdata = FALSE;
            return FALSE;

    return TRUE;

def process_metadata(wpc, wpmd) :
    wps = wpc.stream

    try :
        switch (wpmd.id)

    except case(ID_DUMMY):
        return TRUE;

    except case(ID_DECORR_TERMS):
        return read_decorr_terms(wps, wpmd);

    except case(ID_DECORR_WEIGHTS):
        return read_decorr_weights(wps, wpmd);

    except case(ID_DECORR_SAMPLES):
        return read_decorr_samples(wps, wpmd);

    except case(ID_ENTROPY_VARS):
        return read_entropy_vars(wps, wpmd);

    except case(ID_HYBRID_PROFILE):
        return read_hybrid_profile(wps, wpmd);

    except case(ID_FLOAT_INFO):
        return read_float_info(wps, wpmd);

    except case(ID_INT32_INFO):
        return read_int32_info(wps, wpmd);

    except case(ID_CHANNEL_INFO):
        return read_channel_info(wpc, wpmd);

    except case(ID_SAMPLE_RATE):
        return read_sample_rate(wpc, wpmd);

    except case(ID_CONFIG_BLOCK):
        return read_config_info(wpc, wpmd);

    except case(ID_WV_BITSTREAM):
        return init_wv_bitstream(wpc, wpmd);

    except multicase(ID_SHAPING_WEIGHTS , ID_WVC_BITSTREAM , ID_WVX_BITSTREAM):
        return TRUE;

    except:
        if ((wpmd.id & ID_OPTIONAL_DATA) != 0) :
            return TRUE
        else :
            return FALSE




# This function initializes everything required to unpack a WavPack block
# and must be called before unpack_samples() is called to obtain audio data.
# It is assumed that the WavpackHeader has been read into the wps.wphdr
# (in the current WavpackStream). This is where all the metadata blocks are
# scanned up to the one containing the audio bitstream.

def unpack_init(wpc) :
    wps = wpc.stream;
    wpmd =WavpackMetadata();

    if (wps.wphdr.block_samples > 0 and wps.wphdr.block_index != -1) :
        wps.sample_index = wps.wphdr.block_index;

    wps.mute_error = 0;
    wps.crc = 0xffffffff;
    wps.wvbits.sr = 0;

    while ((read_metadata_buff(wpc, wpmd)) == TRUE) :
        if ((process_metadata(wpc, wpmd)) == FALSE) :
            wpc.error = TRUE;
            wpc.error_message = "invalid metadata!";
            return FALSE;

        if (wpmd.id == ID_WV_BITSTREAM) :
            break;

    
    if (wps.wphdr.block_samples != 0 and (None == wps.wvbits.file) ) :
        wpc.error_message = "invalid WavPack file!";
        wpc.error = TRUE;
        return FALSE;


    if (wps.wphdr.block_samples != 0) :
        if ((wps.wphdr.flags & INT32_DATA) != 0 and wps.int32_sent_bits != 0) :
            wpc.lossy_blocks = 1

        if ((wps.wphdr.flags & FLOAT_DATA) != 0 and (wps.float_flags & (FLOAT_EXCEPTIONS | FLOAT_ZEROS_SENT | FLOAT_SHIFT_SENT | FLOAT_SHIFT_SAME)) != 0) :
            wpc.lossy_blocks = 1

    wpc.error = FALSE;
    wpc.stream = wps;
    return TRUE;


# This function initialzes the main bitstream for audio samples, which must
# be in the "wv" file.

def init_wv_bitstream(wpc, wpmd) :
    wps = wpc.stream;

    if (wpmd.hasdata == TRUE) :
        wps.wvbits = bs_open_read(wpmd.data, 0, wpmd.byte_length, wpc.infile, 0, 0);
    elif (wpmd.byte_length > 0) :
        blen = wpmd.byte_length & 1
        wps.wvbits = bs_open_read(wpc.read_buffer, -1, len(wpc.read_buffer), wpc.infile, (wpmd.byte_length + blen), 1);

    return TRUE;


# Read decorrelation terms from specified metadata block into the
# decorr_passes array. The terms range from -3 to 8, plus 17 & 18;
# other values are reserved and generate errors for now. The delta
# ranges from 0 to 7 with all values valid. Note that the terms are
# stored in the opposite order in the decorr_passes array compared
# to packing.

def read_decorr_terms(wps, wpmd) :
    termcnt = wpmd.byte_length;
    byteptr = wpmd.data;
    tmpwps = WavpackStream();
    
    counter = 0;
    dcounter = 0;

    if (termcnt > MAX_NTERMS) :
        return FALSE
    
    tmpwps.num_terms = termcnt;

    dcounter = termcnt - 1;

    for dcounter in range(termcnt-1,-1,-1) :
        tmpwps.decorr_passes[dcounter].term =   (ord(byteptr[counter]) & 0x1f) - 5
        tmpwps.decorr_passes[dcounter].delta =  (ord(byteptr[counter]) >> 5) & 0x7
        
        counter = counter + 1
    
        if (tmpwps.decorr_passes[dcounter].term < -3 \
            or (tmpwps.decorr_passes[dcounter].term > MAX_TERM and tmpwps.decorr_passes[dcounter].term < 17) \
            or tmpwps.decorr_passes[dcounter].term > 18) :
            return FALSE;

    wps.decorr_passes = tmpwps.decorr_passes;
    wps.num_terms = tmpwps.num_terms;

    return TRUE;


# Read decorrelation weights from specified metadata block into the
# decorr_passes array. The weights range +/-1024, but are rounded and
# truncated to fit in signed chars for metadata storage. Weights are
# separate for the two channels and are specified from the "last" term
# (first during encode). Unspecified weights are set to zero.

def read_decorr_weights(wps, wpmd) :
    termcnt = wpmd.byte_length
    tcount = 0
    byteptr = wpmd.data
    dpp = decorr_pass()
    counter = 0
    dpp_idx = 0
    myiterator = 0

    if ((wps.wphdr.flags & (MONO_FLAG | FALSE_STEREO)) == 0) :
        termcnt /= 2;

    if (termcnt > wps.num_terms) :
        return FALSE;
    
    for tcount in range(wps.num_terms,0,-1) :
        dpp.weight_A = 0
        dpp.weight_B = 0

    myiterator = wps.num_terms;

    while (termcnt > 0) :
        dpp_idx = myiterator - 1
        
        # We need the input to restore_weight to be a signed value
        
        signedCalc1 = ord(byteptr[counter])

        if signedCalc1 & 0x80 == 0x80:
            signedCalc1 = signedCalc1 & 0x7F
            signedCalc1 = signedCalc1 - 0x80
                
        dpp.weight_A = restore_weight(signedCalc1)

        wps.decorr_passes[dpp_idx].weight_A = dpp.weight_A;

        counter = counter + 1

        if ((wps.wphdr.flags & (MONO_FLAG | FALSE_STEREO)) == 0) :
            # We need the input to restore_weight to be a signed value

            signedCalc1 = ord(byteptr[counter])

            if signedCalc1 & 0x80 == 0x80:
                signedCalc1 = signedCalc1 & 0x7F
                signedCalc1 = signedCalc1 - 0x80

            dpp.weight_B = restore_weight( signedCalc1 )
            counter = counter + 1

        wps.decorr_passes[dpp_idx].weight_B = dpp.weight_B

        myiterator = myiterator - 1
        termcnt = termcnt - 1

    dpp_idx = dpp_idx - 1
    while dpp_idx >= 0 :
        wps.decorr_passes[dpp_idx].weight_A = 0
        wps.decorr_passes[dpp_idx].weight_B = 0
        dpp_idx = dpp_idx - 1

    return TRUE;


# Read decorrelation samples from specified metadata block into the
# decorr_passes array. The samples are signed 32-bit values, but are
# converted to signed log2 values for storage in metadata. Values are
# stored for both channels and are specified from the "last" term
# (first during encode) with unspecified samples set to zero. The
# number of samples stored varies with the actual term value, so
# those must obviously come first in the metadata.

def read_decorr_samples(wps, wpmd) :
    byteptr = wpmd.data;
    dpp = decorr_pass();
    tcount = 0
    counter = 0
    dpp_index = 0;
    uns_buf0 = 0
    uns_buf1 = 0
    uns_buf2 = 0
    uns_buf3 = 0
    sample_counter = 0

    for tcount in range(wps.num_terms,0,-1) :
        dpp.term = wps.decorr_passes[dpp_index].term;

        for internalc in range(0,MAX_TERM) :
            dpp.samples_A[internalc] = 0;
            dpp.samples_B[internalc] = 0;
            wps.decorr_passes[dpp_index].samples_A[internalc] = 0;
            wps.decorr_passes[dpp_index].samples_B[internalc] = 0;

        dpp_index = dpp_index + 1

    if (wps.wphdr.version == 0x402 and (wps.wphdr.flags & HYBRID_FLAG) > 0) :
        counter += 2;

        if ((wps.wphdr.flags & (MONO_FLAG | FALSE_STEREO)) == 0) :
            counter += 2;

    dpp_index = dpp_index - 1
    
    while (counter < wpmd.byte_length) :
        if (dpp.term > MAX_TERM) :
            uns_buf0 =  ord(byteptr[counter]) & 0xff
            uns_buf1 =  ord(byteptr[counter + 1]) & 0xff
            uns_buf2 =  ord(byteptr[counter + 2]) & 0xff
            uns_buf3 =  ord(byteptr[counter + 3]) & 0xff

            # We need to convert to 16-bit signed values
            # 0x8000 represents the left most bit in a 16-bit value
            # 0x7fff masks all bits except the leftmost in 16 bits
            
            signedCalc1 = uns_buf0 + (uns_buf1 << 8)
            if signedCalc1 & 0x8000 == 0x8000:
                signedCalc1 = signedCalc1 & 0x7FFF
                signedCalc1 = signedCalc1 - 0x8000

            signedCalc2 = uns_buf2 + (uns_buf3 << 8)
            if signedCalc2 & 0x8000 == 0x8000:
                signedCalc2 = signedCalc2 & 0x7FFF
                signedCalc2 = signedCalc2 - 0x8000

            dpp.samples_A[0] = exp2s( signedCalc1 );
            dpp.samples_A[1] = exp2s( signedCalc2 );
            counter += 4;

            if ((wps.wphdr.flags & (MONO_FLAG | FALSE_STEREO)) == 0) :
                uns_buf0 =  ord(byteptr[counter]) & 0xff
                uns_buf1 =  ord(byteptr[counter + 1]) & 0xff
                uns_buf2 =  ord(byteptr[counter + 2]) & 0xff
                uns_buf3 =  ord(byteptr[counter + 3]) & 0xff

                signedCalc1 = uns_buf0 + (uns_buf1 << 8)
                if signedCalc1 & 0x8000 == 0x8000:
                    signedCalc1 = signedCalc1 & 0x7FFF
                    signedCalc1 = signedCalc1 - 0x8000

                signedCalc2 = uns_buf2 + (uns_buf3 << 8)
                if signedCalc2 & 0x8000 == 0x8000:
                    signedCalc2 = signedCalc2 & 0x7FFF
                    signedCalc2 = signedCalc2 - 0x8000

                dpp.samples_B[0] = exp2s( signedCalc1 )
                dpp.samples_B[1] = exp2s( signedCalc2 )
                counter += 4;

        elif (dpp.term < 0) :
            
            uns_buf0 =  ord(byteptr[counter]) & 0xff
            uns_buf1 =  ord(byteptr[counter + 1]) & 0xff
            uns_buf2 =  ord(byteptr[counter + 2]) & 0xff
            uns_buf3 =  ord(byteptr[counter + 3]) & 0xff

            signedCalc1 = uns_buf0 + (uns_buf1 << 8)
            if signedCalc1 & 0x8000 == 0x8000:
                signedCalc1 = signedCalc1 & 0x7FFF
                signedCalc1 = signedCalc1 - 0x8000
                
            signedCalc2 = uns_buf2 + (uns_buf3 << 8)
            if signedCalc2 & 0x8000 == 0x8000:
                signedCalc2 = signedCalc2 & 0x7FFF
                signedCalc2 = signedCalc2 - 0x8000

            dpp.samples_A[0] = exp2s( signedCalc1 )
            dpp.samples_B[0] = exp2s( signedCalc2 )

            counter += 4;
        else :

            m = 0
            cnt = dpp.term

            while (cnt > 0) :
                uns_buf0 =  ord(byteptr[counter]) & 0xff
                uns_buf1 =  ord(byteptr[counter + 1]) & 0xff

                signedCalc1 = uns_buf0 + (uns_buf1 << 8)
                if signedCalc1 & 0x8000 == 0x8000:
                    signedCalc1 = signedCalc1 & 0x7FFF
                    signedCalc1 = signedCalc1 - 0x8000

                dpp.samples_A[m] = exp2s(signedCalc1)
                counter += 2;

                if ((wps.wphdr.flags & (MONO_FLAG | FALSE_STEREO)) == 0) :
                    uns_buf0 =  ord(byteptr[counter]) & 0xff
                    uns_buf1 =  ord(byteptr[counter + 1]) & 0xff

                    signedCalc1 = uns_buf0 + (uns_buf1 << 8)
                    if signedCalc1 & 0x8000 == 0x8000:
                        signedCalc1 = signedCalc1 & 0x7FFF
                        signedCalc1 = signedCalc1 - 0x8000
                        
                    dpp.samples_B[m] = exp2s( signedCalc1 )
                    counter += 2;

                m = m + 1
                cnt = cnt - 1

        for sample_counter in range(0,MAX_TERM) :
            wps.decorr_passes[dpp_index].samples_A[sample_counter] = dpp.samples_A[sample_counter]
            wps.decorr_passes[dpp_index].samples_B[sample_counter] = dpp.samples_B[sample_counter]

        dpp_index = dpp_index - 1

    return TRUE;


# Read the int32 data from the specified metadata into the specified stream.
# This data is used for integer data that has more than 24 bits of magnitude
# or, in some cases, used to eliminate redundant bits from any audio stream.

def read_int32_info( wps, wpmd) :

    bytecnt = wpmd.byte_length
    byteptr = wpmd.data
    counter = 0

    if (bytecnt != 4) :
        return FALSE

    wps.int32_sent_bits = ord(byteptr[counter])
    counter = counter + 1
    wps.int32_zeros = ord(byteptr[counter])
    counter = counter + 1
    wps.int32_ones = ord(byteptr[counter])
    counter = counter + 1
    wps.int32_dups = ord(byteptr[counter])

    return TRUE;



# Read multichannel information from metadata. The first byte is the total
# number of channels and the following bytes represent the channel_mask
# as described for Microsoft WAVEFORMATEX.

def read_channel_info(wpc, wpmd) :

    bytecnt = wpmd.byte_length
    shift = 0
    byteptr = wpmd.data
    counter = 0
    mask = 0

    if (bytecnt == 0 or bytecnt > 5) :
        return FALSE

    wpc.config.num_channels = ord(byteptr[counter])
    counter = counter + 1

    while (bytecnt >= 0) :
        mask |= (ord(byteptr[counter]) & 0xFF) << shift
        counter = counter + 1
        shift = shift + 8
        bytecnt = bytecnt - 1

    wpc.config.channel_mask = mask
    return TRUE


# Read configuration information from metadata.

def read_config_info(wpc, wpmd) :

    bytecnt = wpmd.byte_length
    byteptr = wpmd.data
    counter = 0

    if (bytecnt >= 3) :
        wpc.config.flags &= 0xff
        wpc.config.flags |= (ord(byteptr[counter]) & 0xFF) << 8
        counter = counter + 1
        wpc.config.flags |= (ord(byteptr[counter]) & 0xFF) << 16
        counter = counter + 1
        wpc.config.flags |= (ord(byteptr[counter]) & 0xFF) << 24

    return TRUE;


# Read non-standard sampling rate from metadata.

def read_sample_rate(wpc, wpmd) :
    bytecnt = wpmd.byte_length
    byteptr = wpmd.data
    counter = 0

    if (bytecnt == 3) :
        wpc.config.sample_rate = ord(byteptr[counter]) & 0xFF
        counter = counter + 1
        wpc.config.sample_rate |= (ord(byteptr[counter]) & 0xFF) << 8
        counter = counter + 1
        wpc.config.sample_rate |= (ord(byteptr[counter]) & 0xFF) << 16
    
    return TRUE



# This monster actually unpacks the WavPack bitstream(s) into the specified
# buffer as 32-bit integers or floats (depending on orignal data). Lossy
# samples will be clipped to their original limits (i.e. 8-bit samples are
# clipped to -128/+127) but are still returned in ints. It is up to the
# caller to potentially reformat this for the final output including any
# multichannel distribution, block alignment or endian compensation. The
# function unpack_init() must have been called and the entire WavPack block
# must still be visible (although wps.blockbuff will not be accessed again).
# For maximum clarity, the function is broken up into segments that handle
# various modes. This makes for a few extra infrequent flag checks, but
# makes the code easier to follow because the nesting does not become so
# deep. For maximum efficiency, the conversion is isolated to tight loops
# that handle an entire buffer. The function returns the total number of
# samples unpacked, which can be less than the number requested if an error
# occurs or the end of the block is reached.

def unpack_samples(wpc, mybuffer, sample_count) :
    wps = wpc.stream;
    flags = wps.wphdr.flags;
    i = 0
    crc = wps.crc


    mute_limit = ((1L << ((flags & MAG_MASK) >> MAG_LSB)) + 2)
    dpp = decorr_pass()
    tcount = 0
    buffer_counter = 0

    samples_processed = 0;

    if (wps.sample_index + sample_count > wps.wphdr.block_index + wps.wphdr.block_samples) :
        sample_count = wps.wphdr.block_index + wps.wphdr.block_samples - wps.sample_index

    if (wps.mute_error > 0) :
        tempc = 0

        if ((flags & MONO_FLAG) > 0) :
            tempc = sample_count
        else :
            tempc = 2 * sample_count

        while (tempc > 0) :
            mybuffer[buffer_counter] = 0;
            tempc = tempc - 1
            buffer_counter = buffer_counter + 1

        wps.sample_index += sample_count;

        return sample_count;

    if ((flags & HYBRID_FLAG) > 0) :
        mute_limit *= 2;


    # ///////////////////// handle version 4 mono data /////////////////////////

    if ((flags & (MONO_FLAG | FALSE_STEREO)) > 0) :
        dpp_index = 0

        i = get_words(sample_count, flags, wps.w, wps.wvbits, mybuffer);

        # System.arraycopy(temp_buffer, 0, mybuffer, 0, sample_count);

        for tcount in range(wps.num_terms - 1,-1,-1) :
            dpp = wps.decorr_passes[dpp_index];
            decorr_mono_pass(dpp, mybuffer, sample_count, buffer_counter);
            dpp_index = dpp_index + 1

        bf_abs = 0

        for q in range(0, sample_count) :
            if mybuffer[q] < 0 :
                bf_abs = -mybuffer[q]
            else :
                bf_abs = mybuffer[q]

            if (bf_abs > mute_limit) :
                i = q
                break
            
            crcstep1 = (crc * 3) & 0xffffffff
            crc = (crcstep1 + mybuffer[q]) & 0xffffffff
 
            # crc = crc * 3 + mybuffer[q];


    # //////////////////// handle version 4 stereo data ////////////////////////

    else :
        
        samples_processed = get_words(sample_count, flags, wps.w, wps.wvbits, mybuffer);

        i = samples_processed;

        if (sample_count < 16) :
            dpp_index = 0

            for tcount in range(wps.num_terms - 1, -1, -1) :
                dpp = wps.decorr_passes[dpp_index];
                decorr_stereo_pass(dpp, mybuffer, sample_count, buffer_counter);
                wps.decorr_passes[dpp_index] = dpp;
                dpp_index = dpp_index + 1
        else :
            dpp_index = 0;

            for tcount in range(wps.num_terms - 1, -1, -1) :

                dpp = wps.decorr_passes[dpp_index]

                decorr_stereo_pass(dpp, mybuffer, 8, buffer_counter)

                decorr_stereo_pass_cont(dpp, mybuffer, sample_count - 8, buffer_counter + 16)
                wps.decorr_passes[dpp_index] = dpp

                dpp_index = dpp_index + 1

        if ((flags & JOINT_STEREO) > 0) :
            bf_abs = 0
            bf1_abs = 0

            for buffer_counter in range(0,sample_count * 2,2) :

                mybuffer[buffer_counter + 1] = mybuffer[buffer_counter + 1] - (mybuffer[buffer_counter] >> 1)
                mybuffer[buffer_counter] = mybuffer[buffer_counter] + mybuffer[buffer_counter + 1]

                if mybuffer[buffer_counter] < 0 :
                    bf_abs = -mybuffer[buffer_counter]
                else :
                    bf_abs = mybuffer[buffer_counter]
                
                if mybuffer[buffer_counter + 1] < 0 :
                    bf1_abs = -mybuffer[buffer_counter + 1]
                else :
                    bf1_abs = mybuffer[buffer_counter + 1]

                if (bf_abs > mute_limit or bf1_abs > mute_limit) :
                    i = buffer_counter / 2
                    break

                crcstep1 = (crc * 3) & 0xffffffff
                crcstep2 = (crcstep1 + mybuffer[buffer_counter]) & 0xffffffff
                crcstep3 = (crcstep2 * 3) & 0xffffffff

                crc = (crcstep3 + mybuffer[buffer_counter + 1] ) & 0xffffffff

        else :
            bf_abs = 0
            bf1_abs = 0

            for buffer_counter in range(0,sample_count * 2,2) :
                if mybuffer[buffer_counter] < 0 :
                    bf_abs = -mybuffer[buffer_counter]
                else :
                    bf_abs = mybuffer[buffer_counter]
                    
                if mybuffer[buffer_counter + 1] < 0 :
                    bf1_abs = -mybuffer[buffer_counter + 1]
                else :
                    bf1_abs = mybuffer[buffer_counter + 1]

                if (bf_abs > mute_limit or bf1_abs > mute_limit) :
                    i = buffer_counter / 2;
                    break

                crcstep1 = (crc * 3) & 0xffffffff
                crcstep2 = (crcstep1 + mybuffer[buffer_counter]) & 0xffffffff
                crcstep3 = (crcstep2 * 3) & 0xffffffff

                crc = (crcstep3 + mybuffer[buffer_counter + 1] ) & 0xffffffff

    if (i != sample_count) :
        sc = 0
       
        if ((flags & MONO_FLAG) > 0) :
            sc = sample_count
        else :
            sc = 2 * sample_count
            
        buffer_counter = 0

        while (sc > 0) :
            mybuffer[buffer_counter] = 0
            sc = sc -1
            buffer_counter = buffer_counter + 1

        wps.mute_error = 1
        i = sample_count

    mybuffer = fixup_samples(wps, mybuffer, i);

    if ((flags & FALSE_STEREO) > 0) :
        dest_idx = i * 2;
        src_idx = i;
        c = i;

        dest_idx = dest_idx - 1
        src_idx = src_idx - 1

        while (c > 0) :
            mybuffer[dest_idx] = mybuffer[src_idx];
            dest_idx = dest_idx - 1
            mybuffer[dest_idx] = mybuffer[src_idx];
            dest_idx = dest_idx - 1
            src_idx = src_idx - 1
            c = c -1

    wps.sample_index += i
    wps.crc = crc

    return i;


def decorr_stereo_pass(dpp, mybuffer, sample_count, buf_idx) :
    delta = dpp.delta;
    weight_A = dpp.weight_A;
    weight_B = dpp.weight_B;
    sam_A = 0
    sam_B = 0
    m = 0
    k = 0
    bptr_counter = 0

    try :
        switch (dpp.term)
    except case(17) :
        for bptr_counter in range(buf_idx, buf_idx + sample_count * 2, 2) :
            sam_A = 2 * dpp.samples_A[0] - dpp.samples_A[1];
            dpp.samples_A[1] = dpp.samples_A[0];
            dpp.samples_A[0] = ((weight_A * sam_A + 512) >> 10) + mybuffer[bptr_counter];

            if (sam_A != 0 and mybuffer[bptr_counter] != 0) :
                if ((sam_A ^ mybuffer[bptr_counter]) < 0) :
                    weight_A = weight_A - delta;
                else :
                    weight_A = weight_A + delta;

            mybuffer[bptr_counter] = dpp.samples_A[0];

            sam_A = 2 * dpp.samples_B[0] - dpp.samples_B[1];
            dpp.samples_B[1] = dpp.samples_B[0];
            dpp.samples_B[0] =  ((weight_B *sam_A + 512) >> 10) + mybuffer[bptr_counter + 1];

            if (sam_A != 0 and mybuffer[bptr_counter + 1] != 0) :
                if ((sam_A ^ mybuffer[bptr_counter + 1]) < 0) :
                    weight_B = weight_B - delta;
                else :
                    weight_B = weight_B + delta;

            mybuffer[bptr_counter + 1] = dpp.samples_B[0]

    except case(18):
        
        for bptr_counter in range(buf_idx, buf_idx + sample_count * 2, 2) :
            sam_A = (3 * dpp.samples_A[0] - dpp.samples_A[1]) >> 1;
            dpp.samples_A[1] = dpp.samples_A[0];
            dpp.samples_A[0] =  ((weight_A * sam_A + 512) >> 10) + mybuffer[bptr_counter];

            if (sam_A != 0 and mybuffer[bptr_counter] != 0) :
                if ((sam_A ^ mybuffer[bptr_counter]) < 0) :
                    weight_A = weight_A - delta;
                else :
                    weight_A = weight_A + delta;

            mybuffer[bptr_counter] = dpp.samples_A[0];

            sam_A = (3 * dpp.samples_B[0] - dpp.samples_B[1]) >> 1;
            dpp.samples_B[1] = dpp.samples_B[0];
            dpp.samples_B[0] = ((weight_B * sam_A + 512) >> 10) + mybuffer[bptr_counter + 1];

            if (sam_A != 0 and mybuffer[bptr_counter + 1] != 0) :
                if ((sam_A ^ mybuffer[bptr_counter + 1]) < 0) :
                    weight_B = weight_B - delta;
                else :
                    weight_B = weight_B + delta;

            mybuffer[bptr_counter + 1] = dpp.samples_B[0]

    except case(-1) :
        for bptr_counter in range(buf_idx, buf_idx + sample_count * 2, 2) :
            sam_A = mybuffer[bptr_counter] + ((weight_A * dpp.samples_A[0] + 512) >> 10);

            if ((dpp.samples_A[0] ^ mybuffer[bptr_counter]) < 0) :
                if (dpp.samples_A[0] != 0 and mybuffer[bptr_counter] != 0 ) :
                    weight_A = weight_A - delta
                    if weight_A < -1024:
                        weight_A = -1024
            else :
                if (dpp.samples_A[0] != 0 and mybuffer[bptr_counter] != 0 ) :
                    weight_A = weight_A + delta
                    if weight_A > 1024 :
                        weight_A = 1024

            mybuffer[bptr_counter] = sam_A;
            dpp.samples_A[0] = mybuffer[bptr_counter + 1] +  ((weight_B * sam_A + 512) >> 10);

            if ((sam_A ^ mybuffer[bptr_counter + 1]) < 0) :
                if (sam_A != 0 and mybuffer[bptr_counter + 1] != 0 ) :
                    weight_B = weight_B - delta
                    if weight_B < -1024 :
                        weight_B = -1024
            else :
                if (sam_A != 0 and mybuffer[bptr_counter + 1] != 0 ) :
                    weight_B = weight_B + delta
                    if weight_B > 1024 :
                        weight_B = 1024

            mybuffer[bptr_counter + 1] = dpp.samples_A[0]

    except case(-2) :
        sam_B = 0;
        sam_A = 0;

        for bptr_counter in range(buf_idx, buf_idx + sample_count * 2, 2) :
            sam_B = mybuffer[bptr_counter + 1] +((weight_B * dpp.samples_B[0] + 512) >> 10)

            if ((dpp.samples_B[0] ^ mybuffer[bptr_counter + 1]) < 0) :
                if (dpp.samples_B[0] != 0 and mybuffer[bptr_counter + 1] != 0 ) :
                    weight_B = weight_B - delta
                    if weight_B < -1024 :
                        weight_B = -1024
            else :
                if (dpp.samples_B[0] != 0 and mybuffer[bptr_counter + 1] != 0 ) :
                    weight_B = weight_B + delta
                    if weight_B > 1024 :
                        weight_B = 1024

            mybuffer[bptr_counter + 1] = sam_B

            dpp.samples_B[0] = mybuffer[bptr_counter] + ((weight_A * sam_B + 512) >> 10)

            if ((sam_B ^ mybuffer[bptr_counter]) < 0) :
                if (sam_B != 0 and mybuffer[bptr_counter] != 0 ) :
                    weight_A = weight_A - delta
                    if weight_A < -1024 :
                        weight_A = -1024
            else :
                if (sam_B != 0 and mybuffer[bptr_counter] != 0 ) :
                    weight_A = weight_A + delta
                    if weight_A > 1024 :
                        weight_A = 1024

            mybuffer[bptr_counter] = dpp.samples_B[0];


    except case(-3) :
        sam_A = 0;

        for bptr_counter in range(buf_idx, buf_idx + sample_count * 2, 2) :
            sam_A = mybuffer[bptr_counter] + ((weight_A * dpp.samples_A[0] + 512) >> 10)

            if ((dpp.samples_A[0] ^ mybuffer[bptr_counter]) < 0) :
                if (dpp.samples_A[0] != 0 and mybuffer[bptr_counter] != 0 ) :
                    weight_A = weight_A - delta
                    if weight_A < -1024 :
                        weight_A = -1024
            else :
                if (dpp.samples_A[0] != 0 and mybuffer[bptr_counter] != 0 ) :
                    weight_A = weight_A + delta
                    if weight_A > 1024 :
                        weight_A = 1024

            sam_B = mybuffer[bptr_counter + 1] + ((weight_B * dpp.samples_B[0] + 512) >> 10)

            if ((dpp.samples_B[0] ^ mybuffer[bptr_counter + 1]) < 0) :
                if (dpp.samples_B[0] != 0 and mybuffer[bptr_counter + 1] != 0 ) :
                    weight_B = weight_B - delta
                    if weight_B < -1024 :
                        weight_B = -1024
            else :
                if (dpp.samples_B[0] != 0 and mybuffer[bptr_counter + 1] != 0 ) :
                    weight_B = weight_B + delta
                    if weight_B > 1024 :
                        weight_B = 1024

            mybuffer[bptr_counter] = dpp.samples_B[0] = sam_A
            mybuffer[bptr_counter + 1] = dpp.samples_A[0] = sam_B

    except :

        sam_A = 0
        m = 0
        k = dpp.term & (MAX_TERM - 1)

        for bptr_counter in range(buf_idx, buf_idx + sample_count * 2, 2) :
            sam_A = dpp.samples_A[m];
            dpp.samples_A[k] = ((weight_A * sam_A + 512) >> 10) + mybuffer[bptr_counter];

            if (sam_A != 0 and mybuffer[bptr_counter] != 0) :
                if ((sam_A ^ mybuffer[bptr_counter]) < 0) :
                    weight_A = weight_A - delta
                else :
                    weight_A = weight_A + delta

            mybuffer[bptr_counter] = dpp.samples_A[k];

            sam_A = dpp.samples_B[m];
            dpp.samples_B[k] = ((weight_B * sam_A + 512) >> 10) + mybuffer[bptr_counter + 1];
   
            if (sam_A != 0 and mybuffer[bptr_counter + 1] != 0) :
                if ((sam_A ^ mybuffer[bptr_counter + 1]) < 0) :
                    weight_B = weight_B - delta
                else :
                    weight_B = weight_B + delta

            mybuffer[bptr_counter + 1] = dpp.samples_B[k];

            m = (m + 1) & (MAX_TERM - 1)
            k = (k + 1) & (MAX_TERM - 1)

        if (m != 0) :
            temp_samples = [0] * MAX_TERM

            for t in range(0, len(dpp.samples_A)) :
                temp_samples[t] = dpp.samples_A[t]

            for k in range(0,MAX_TERM) :
                dpp.samples_A[k] = temp_samples[m & (MAX_TERM - 1)]
                m = m + 1

            for tmpiter in range(0,MAX_TERM) :
                temp_samples[tmpiter] = dpp.samples_B[tmpiter]

            for k in range(0,MAX_TERM) :
                dpp.samples_B[k] = temp_samples[m & (MAX_TERM - 1)]
                m = m + 1

    dpp.weight_A =  weight_A;
    dpp.weight_B =  weight_B;



def decorr_stereo_pass_cont(dpp, mybuffer, sample_count, buf_idx) :
    delta = dpp.delta
    weight_A = dpp.weight_A
    weight_B = dpp.weight_B
    tptr = 0
    sam_A = 0
    sam_B = 0
    k = 0
    i = 0
    buffer_index = buf_idx
    end_index = buf_idx + sample_count * 2

    
    try :
        switch (dpp.term)
    except case(17) :
        for buffer_index in range(buf_idx, end_index, 2) :
            sam_A = 2 * mybuffer[buffer_index - 2] - mybuffer[buffer_index - 4]

            sam_B = mybuffer[buffer_index]
            mybuffer[buffer_index] = ((weight_A *  sam_A + 512) >> 10) + sam_B

            if (sam_A != 0 and sam_B != 0) : 
                if (sam_A ^ sam_B) < 0 :
                    weight_A -= delta; 
                else :
                    weight_A += delta;

            sam_A = 2 * mybuffer[buffer_index - 1] - mybuffer[buffer_index - 3]
            sam_B = mybuffer[buffer_index + 1]
            mybuffer[buffer_index + 1] = ((weight_B * sam_A + 512) >> 10) + sam_B

            if (sam_A != 0 and sam_B != 0) : 
                if (sam_A ^ sam_B) < 0 :
                    weight_B -= delta; 
                else :
                    weight_B += delta;            
            

        buffer_index = end_index
        
        dpp.samples_B[0] = mybuffer[buffer_index - 1]
        dpp.samples_A[0] = mybuffer[buffer_index - 2]
        dpp.samples_B[1] = mybuffer[buffer_index - 3]
        dpp.samples_A[1] = mybuffer[buffer_index - 4]

    except case(18) :
        for buffer_index in range(buf_idx, end_index, 2) :
            sam_A = (3 * mybuffer[buffer_index - 2] - mybuffer[buffer_index - 4]) >> 1;
            sam_B = mybuffer[buffer_index]
            mybuffer[buffer_index] = ((weight_A * sam_A + 512) >> 10) + sam_B

            if (sam_A != 0 and sam_B != 0) : 
                if (sam_A ^ sam_B) < 0 :
                    weight_A -= delta; 
                else :
                    weight_A += delta;


            sam_A = (3 * mybuffer[buffer_index - 1] - mybuffer[buffer_index - 3]) >> 1
            sam_B = mybuffer[buffer_index + 1]
            mybuffer[buffer_index + 1] = ((weight_B * sam_A + 512) >> 10) + sam_B

            if (sam_A != 0 and sam_B != 0) : 
                if (sam_A ^ sam_B) < 0 :
                    weight_B -= delta; 
                else :
                    weight_B += delta;

        buffer_index = end_index

        dpp.samples_B[0] = mybuffer[buffer_index - 1]
        dpp.samples_A[0] = mybuffer[buffer_index - 2]
        dpp.samples_B[1] = mybuffer[buffer_index - 3]
        dpp.samples_A[1] = mybuffer[buffer_index - 4]


    except case(-1) :
        for buffer_index in range(buf_idx, end_index, 2) :
            sam_A = mybuffer[buffer_index]

            mybuffer[buffer_index] = ((weight_A * mybuffer[buffer_index - 1] + 512) >> 10) + sam_A

            if ((mybuffer[buffer_index - 1] ^ sam_A) < 0) :
                if (mybuffer[buffer_index - 1] != 0 and sam_A != 0 ) :
                    weight_A = weight_A - delta
                    if weight_A < -1024 :
                        weight_A = -1024
            else :
                if (mybuffer[buffer_index - 1] != 0 and sam_A != 0 ) :
                    weight_A = weight_A + delta
                    if (weight_A > 1024) :
                        weight_A = 1024

            sam_A = mybuffer[buffer_index + 1]
            mybuffer[buffer_index + 1] = ((weight_B *  mybuffer[buffer_index] + 512) >> 10) + sam_A

            if ((mybuffer[buffer_index] ^ sam_A) < 0) :
                if (mybuffer[buffer_index] != 0 and sam_A != 0 ) :
                    weight_B = weight_B - delta
                    if weight_B < -1024 :
                        weight_B = -1024
            else :
                if (mybuffer[buffer_index] != 0 and sam_A != 0 ) :
                    weight_B = weight_B + delta
                    if weight_B > 1024 :
                        weight_B = 1024

        buffer_index = end_index
        
        dpp.samples_A[0] = mybuffer[buffer_index - 1]

    except case(-2):
        sam_A = 0;
        sam_B = 0;

        for buffer_index in range(buf_idx, end_index, 2) :
            sam_A = mybuffer[buffer_index + 1]
            mybuffer[buffer_index + 1] =((weight_B * mybuffer[buffer_index - 2] + 512) >> 10) + sam_A

            if ((mybuffer[buffer_index - 2] ^ sam_A) < 0) :
                if (mybuffer[buffer_index - 2] != 0 and sam_A != 0 ) :
                    weight_B = weight_B - delta
                    if weight_B < -1024 :
                        weight_B = -1024
            else :
                if (mybuffer[buffer_index - 2] != 0 and sam_A != 0 ) :
                    weight_B = weight_B + delta
                    if weight_B > 1024 :
                        weight_B = 1024


            sam_A = mybuffer[buffer_index]
            mybuffer[buffer_index] = ((weight_A * mybuffer[buffer_index + 1] + 512) >> 10) + sam_A

            if ((mybuffer[buffer_index + 1] ^ sam_A) < 0) :
                if (mybuffer[buffer_index + 1] != 0 and sam_A != 0) :
                    weight_A = weight_A - delta
                    if weight_A < -1024 :
                        weight_A = -1024
            else :
                if (mybuffer[buffer_index + 1] != 0 and sam_A != 0) :
                    weight_A = weight_A + delta
                    if (weight_A > 1024) :
                        weight_A = 1024


        buffer_index = end_index
        
        dpp.samples_B[0] = mybuffer[buffer_index - 2];

    except case(-3) :
        for buffer_index in range(buf_idx, end_index, 2) :
            sam_A = mybuffer[buffer_index]

            mybuffer[buffer_index] = ((weight_A * mybuffer[buffer_index - 1] + 512) >> 10) + sam_A

            if ((mybuffer[buffer_index - 1] ^ sam_A) < 0) :
                if (mybuffer[buffer_index - 1] != 0 and sam_A != 0 ) :
                    weight_A = weight_A - delta
                    if weight_A < -1024 :
                        weight_A = -1024
            else :
                if (mybuffer[buffer_index - 1] != 0 and sam_A != 0 ) :
                    weight_A = weight_A + delta
                    if weight_A > 1024 :
                        weight_A = 1024

            sam_A = mybuffer[buffer_index + 1]
            mybuffer[buffer_index + 1] = ((weight_B * mybuffer[buffer_index - 2] + 512) >> 10) + sam_A

            if ((mybuffer[buffer_index - 2] ^ sam_A) < 0) :
                if (mybuffer[buffer_index - 2] != 0 and sam_A != 0 ) :
                    weight_B = weight_B - delta
                    if weight_B < -1024 :
                        weight_B = -1024
            else :
                if (mybuffer[buffer_index - 2] != 0 and sam_A != 0 ) :
                    weight_B = weight_B + delta
                    if weight_B > 1024 :
                        weight_B = 1024

        buffer_index = end_index
        
        dpp.samples_A[0] = mybuffer[buffer_index - 1];
        dpp.samples_B[0] = mybuffer[buffer_index - 2];

    except :
        tptr = buf_idx - (dpp.term * 2);

        for buffer_index in range(buf_idx, end_index, 2) :
            sam_A = mybuffer[buffer_index]
            
            mybuffer[buffer_index] = ((weight_A * mybuffer[tptr] + 512) >> 10) + sam_A

            if (mybuffer[tptr] != 0 and sam_A != 0) : 
                if (mybuffer[tptr] ^ sam_A) < 0 :
                    weight_A -= delta; 
                else :
                    weight_A += delta;


            sam_A = mybuffer[buffer_index + 1]
            mybuffer[buffer_index + 1] = ((weight_B * mybuffer[tptr + 1] + 512) >> 10) + sam_A

            if (mybuffer[tptr + 1] != 0 and sam_A != 0) : 
                if (mybuffer[tptr + 1] ^ sam_A) < 0 :
                    weight_B -= delta; 
                else :
                    weight_B += delta;


            tptr += 2;

        buffer_index = end_index - 1

        k = dpp.term - 1
        i = 8
        while i > 0 :
            i = i - 1
            dpp.samples_B[k & (MAX_TERM - 1)] = mybuffer[buffer_index]
            buffer_index = buffer_index - 1
            dpp.samples_A[k & (MAX_TERM - 1)] = mybuffer[buffer_index]
            buffer_index = buffer_index - 1
            k = k - 1

    dpp.weight_A =  weight_A;
    dpp.weight_B =  weight_B;



def decorr_mono_pass(dpp, mybuffer, sample_count,  buf_idx) :
    delta = dpp.delta
    weight_A = dpp.weight_A
    sam_A = 0
    m = 0
    k = 0
    bptr_counter = 0
    end_index = buf_idx + sample_count

    try :
        switch (dpp.term)
    except case(17) :

        for bptr_counter in range(buf_idx, end_index, 1) :
            sam_A = 2 * dpp.samples_A[0] - dpp.samples_A[1]
            dpp.samples_A[1] = dpp.samples_A[0]
            dpp.samples_A[0] =  ((weight_A *sam_A + 512) >> 10) + mybuffer[bptr_counter]

            if (sam_A != 0 and mybuffer[bptr_counter] != 0) :
                if ((sam_A ^ mybuffer[bptr_counter]) < 0) :
                    weight_A = weight_A - delta;
                else :
                    weight_A = weight_A + delta
                    
            mybuffer[bptr_counter] = dpp.samples_A[0];


    except case (18) :
        for bptr_counter in range(buf_idx, end_index, 1) :
            sam_A = (3 * dpp.samples_A[0] - dpp.samples_A[1]) >> 1
            dpp.samples_A[1] = dpp.samples_A[0]
            dpp.samples_A[0] = ((weight_A * sam_A + 512) >> 10) + mybuffer[bptr_counter]

            if (sam_A != 0 and mybuffer[bptr_counter] != 0) :
                if ((sam_A ^ mybuffer[bptr_counter]) < 0) :
                    weight_A = weight_A - delta
                else :
                    weight_A = weight_A + delta

            mybuffer[bptr_counter] = dpp.samples_A[0];

    except :
        m = 0
        k = dpp.term & (MAX_TERM - 1)
        

        for bptr_counter in range(buf_idx, end_index, 1) :
            sam_A = dpp.samples_A[m]
            dpp.samples_A[k] = ((weight_A * sam_A + 512) >> 10) + mybuffer[bptr_counter]

            if (sam_A != 0 and mybuffer[bptr_counter] != 0) :
                if ((sam_A ^ mybuffer[bptr_counter]) < 0) :
                    weight_A = weight_A - delta
                else :
                    weight_A = weight_A + delta

            mybuffer[bptr_counter] = dpp.samples_A[k];
            m = (m + 1) & (MAX_TERM - 1);
            k = (k + 1) & (MAX_TERM - 1);

        if (m != 0) :
            temp_samples = [0] * MAX_TERM

            for tmpiter in range(0,MAX_TERM) :
                temp_samples[tmpiter] = dpp.samples_A[tmpiter]

            for k in range(0,MAX_TERM) :
                dpp.samples_A[k] = temp_samples[m & (MAX_TERM - 1)]
                m = m + 1

    dpp.weight_A =  weight_A


# This is a helper function for unpack_samples() that applies several final
# operations. First, if the data is 32-bit float data, then that conversion
# is done by float_values() (whether lossy or lossless) and we return.
# Otherwise, if the extended integer data applies, then that operation is
# executed first. If the unpacked data is lossy (and not corrected) then
# it is clipped and shifted in a single operation. Otherwise, if it's
# lossless then the last step is to apply the final shift (if any).

def fixup_samples( wps, mybuffer, sample_count) :
    flags = wps.wphdr.flags
    shift = (flags & SHIFT_MASK) >> SHIFT_LSB

    if ((flags & FLOAT_DATA) > 0) :
        sc = 0

        if ((flags & MONO_FLAG) > 0) :
            sc = sample_count
        else :
            sc = sample_count * 2

        mybuffer = float_values(wps, mybuffer, sc)

    if ((flags & INT32_DATA) > 0) :

        sent_bits = wps.int32_sent_bits
        zeros = wps.int32_zeros;
        ones = wps.int32_ones
        dups = wps.int32_dups
        buffer_counter = 0
        count = 0

        if ((flags & MONO_FLAG) > 0) :
            count = sample_count
        else :
            count = sample_count * 2

        if ((flags & HYBRID_FLAG) == 0 and sent_bits == 0 and (zeros + ones + dups) != 0) :
            while (count > 0) :
                if (zeros != 0) :
                    mybuffer[buffer_counter] <<= zeros;

                elif (ones != 0) :
                    mybuffer[buffer_counter] = ((mybuffer[buffer_counter] + 1) << ones) - 1;

                elif (dups != 0) :
                    mybuffer[buffer_counter] = ((mybuffer[buffer_counter] + (mybuffer[buffer_counter] & 1)) << dups) - (mybuffer[buffer_counter] & 1);

                buffer_counter = buffer_counter + 1
                count = count - 1
        else :
            shift += zeros + sent_bits + ones + dups;


    if ((flags & HYBRID_FLAG) > 0) :
        min_value = 0
        max_value = 0
        min_shifted = 0
        max_shifted = 0
        buffer_counter = 0

        try :
            switch (flags & BYTES_STORED)
        except case(0) :
            min_value = -128
            min_shifted = (min_value >> shift) << shift
            max_value = 127
            max_shifted = (max_value >> shift) << shift

        except case (1):
            min_value = -32768
            min_shifted = (min_value >> shift) << shift
            max_value = 32767
            max_shifted = (max_value >> shift) << shift

        except case (2) :
            min_value = -8388608
            min_shifted = (min_value >> shift) << shift
            max_value = 8388607
            max_shifted = (max_value >> shift) << shift

        except case (3):
            pass
            
        except :
            min_value = 0x80000000
            min_shifted = (min_value >> shift) << shift
            max_value = 0x7FFFFFFF
            max_shifted = (max_value >> shift) << shift

        if ((flags & MONO_FLAG) == 0) :
            sample_count *= 2

        while (sample_count > 0) :
            if (mybuffer[buffer_counter] < min_value) :
                mybuffer[buffer_counter] = min_shifted

            elif (mybuffer[buffer_counter] > max_value) :
                mybuffer[buffer_counter] = max_shifted

            else :
                mybuffer[buffer_counter] <<= shift

            buffer_counter = buffer_counter + 1
            sample_count = sample_count - 1

    elif (shift != 0) :
        buffer_counter = 0;

        if ((flags & MONO_FLAG) == 0) :
            sample_count *= 2;

        while (sample_count > 0) :
            mybuffer[buffer_counter] = mybuffer[buffer_counter] << shift;
            buffer_counter = buffer_counter + 1
            sample_count = sample_count - 1

    return mybuffer


# This function checks the crc value(s) for an unpacked block, returning the
# number of actual crc errors detected for the block. The block must be
# completely unpacked before this test is valid. For losslessly unpacked
# blocks of float or extended integer data the extended crc is also checked.
# Note that WavPack's crc is not a CCITT approved polynomial algorithm, but
# is a much simpler method that is virtually as robust for real world data.

def check_crc_error(wpc) :
    wps = wpc.stream;
    result = 0;

    if (wps.crc != wps.wphdr.crc) :
        result = result + 1

    return result;


# Read the median log2 values from the specifed metadata structure, convert
# them back to 32-bit unsigned values and store them. If length is not
# exactly correct then we flag and return an error.

def read_entropy_vars(wps, wpmd) :
    byteptr = wpmd.data # byteptr needs to be unsigned chars, so convert to int array
    b_array = [0] * 12
    i = 0;
    w = words_data()

    for i in range(0,6) :
        b_array[i] = ord(byteptr[i]) & 0xff

    w.holding_one = 0;
    w.holding_zero = 0;

    if (wpmd.byte_length != 12) :
        if ((wps.wphdr.flags & (MONO_FLAG | FALSE_STEREO)) == 0) :
            return FALSE;

    w.c[0].median[0] = exp2s(b_array[0] + (b_array[1] << 8));
    w.c[0].median[1] = exp2s(b_array[2] + (b_array[3] << 8));
    w.c[0].median[2] = exp2s(b_array[4] + (b_array[5] << 8));

    if ((wps.wphdr.flags & (MONO_FLAG | FALSE_STEREO)) == 0) :
        for i in range(6,12) :
            b_array[i] = ord(byteptr[i]) & 0xff

        w.c[1].median[0] = exp2s(b_array[6] + (b_array[7] << 8));
        w.c[1].median[1] = exp2s(b_array[8] + (b_array[9] << 8));
        w.c[1].median[2] = exp2s(b_array[10] + (b_array[11] << 8));

    wps.w = w

    return TRUE


# Read the hybrid related values from the specifed metadata structure, convert
# them back to their internal formats and store them. The extended profile
# stuff is not implemented yet, so return an error if we get more data than
# we know what to do with.

def read_hybrid_profile(wps, wpmd) :
    byteptr = wpmd.data
    bytecnt = wpmd.byte_length
    buffer_counter = 0
    uns_buf = 0
    uns_buf_plusone = 0

    if ((wps.wphdr.flags & HYBRID_BITRATE) != 0) :
        uns_buf = ord(byteptr[buffer_counter]) & 0xff
        uns_buf_plusone =  ord(byteptr[buffer_counter + 1]) & 0xff

        wps.w.c[0].slow_level = exp2s(uns_buf + (uns_buf_plusone << 8))
        buffer_counter = buffer_counter + 2;

        if ((wps.wphdr.flags & (MONO_FLAG | FALSE_STEREO)) == 0) :
            uns_buf = ord(byteptr[buffer_counter]) & 0xff
            uns_buf_plusone = ord(byteptr[buffer_counter + 1]) & 0xff
            wps.w.c[1].slow_level = exp2s(uns_buf + (uns_buf_plusone << 8))
            buffer_counter = buffer_counter + 2


    uns_buf = ord(byteptr[buffer_counter]) & 0xff
    uns_buf_plusone = ord(byteptr[buffer_counter + 1]) & 0xff

    wps.w.bitrate_acc[0] = (uns_buf + (uns_buf_plusone << 8)) << 16
    buffer_counter = buffer_counter + 2

    if ((wps.wphdr.flags & (MONO_FLAG | FALSE_STEREO)) == 0) :
        uns_buf = ord(byteptr[buffer_counter]) & 0xff
        uns_buf_plusone = ord(byteptr[buffer_counter + 1]) & 0xff

        wps.w.bitrate_acc[1] =(uns_buf + (uns_buf_plusone << 8)) << 16
        buffer_counter = buffer_counter + 2

    if (buffer_counter < bytecnt) :
        uns_buf = ord(byteptr[buffer_counter]) & 0xff
        uns_buf_plusone = ord(byteptr[buffer_counter + 1]) & 0xff

        wps.w.bitrate_delta[0] = exp2s((uns_buf + (uns_buf_plusone << 8)))
        buffer_counter = buffer_counter + 2;

        if ((wps.wphdr.flags & (MONO_FLAG | FALSE_STEREO)) == 0) :
            uns_buf = ord(byteptr[buffer_counter]) & 0xff
            uns_buf_plusone = ord(byteptr[buffer_counter + 1]) & 0xff
            wps.w.bitrate_delta[1] = exp2s((uns_buf + (uns_buf_plusone << 8)))
            buffer_counter = buffer_counter + 2

        if (buffer_counter < bytecnt) :
            return FALSE

    else :
        wps.w.bitrate_delta[0] = wps.w.bitrate_delta[1] = 0
 
    return TRUE


# This function is called during both encoding and decoding of hybrid data to
# update the "error_limit" variable which determines the maximum sample error
# allowed in the main bitstream. In the HYBRID_BITRATE mode (which is the only
# currently implemented) this is calculated from the slow_level values and the
# bitrate accumulators. Note that the bitrate accumulators can be changing.

def update_error_limit(w, flags) :

    w.bitrate_acc[0] = w.bitrate_acc[0] + w.bitrate_delta[0] 
    bitrate_0 = w.bitrate_acc[0] >> 16

    if ((flags & (MONO_FLAG | FALSE_STEREO)) != 0) :
        if ((flags & HYBRID_BITRATE) != 0) :
            slow_log_0 = ((w.c[0].slow_level + SLO) >> SLS)

            if (slow_log_0 - bitrate_0 > -0x100) :
                w.c[0].error_limit = exp2s(slow_log_0 - bitrate_0 + 0x100);
            else :
                w.c[0].error_limit = 0;
        else :
            w.c[0].error_limit = exp2s(bitrate_0);
    else :
        w.bitrate_acc[1] = w.bitrate_acc[1] + w.bitrate_delta[1]
        bitrate_1 = w.bitrate_acc[1] >> 16

        if ((flags & HYBRID_BITRATE) != 0) :
            slow_log_0 = ((w.c[0].slow_level + SLO) >> SLS)
            slow_log_1 = ((w.c[1].slow_level + SLO) >> SLS)

            if ((flags & HYBRID_BALANCE) != 0) :
                balance = (slow_log_1 - slow_log_0 + bitrate_1 + 1) >> 1

                if (balance > bitrate_0) :
                    bitrate_1 = bitrate_0 * 2
                    bitrate_0 = 0
                elif (-balance > bitrate_0) :
                    bitrate_0 = bitrate_0 * 2
                    bitrate_1 = 0
                else :
                    bitrate_1 = bitrate_0 + balance
                    bitrate_0 = bitrate_0 - balance


            if (slow_log_0 - bitrate_0 > -0x100) :
                w.c[0].error_limit = exp2s(slow_log_0 - bitrate_0 + 0x100)
            else :
                w.c[0].error_limit = 0;

            if (slow_log_1 - bitrate_1 > -0x100) :
                w.c[1].error_limit = exp2s(slow_log_1 - bitrate_1 + 0x100)
            else :
                w.c[1].error_limit = 0
            
        else :
            w.c[0].error_limit = exp2s(bitrate_0)
            w.c[1].error_limit = exp2s(bitrate_1)

    return w


# Read the next word from the bitstream "wvbits" and return the value. This
# function can be used for hybrid or lossless streams, but since an
# optimized version is available for lossless this function would normally
# be used for hybrid only. If a hybrid lossless stream is being read then
# the "correction" offset is written at the specified pointer. A return value
# of WORD_EOF indicates that the end of the bitstream was reached (all 1s) or
# some other error occurred.

def get_words(nsamples, flags, w, bs, buffer) :
    c = w.c;
    csamples = 0
    buffer_counter = 0;
    entidx = 1;

    if ((flags & (MONO_FLAG | FALSE_STEREO)) == 0) : # if not mono
        nsamples *= 2;
    else :
        # it is mono
        entidx = 0;

    for csamples in range(0, nsamples) :

        ones_count = 0
        low = 0
        mid = 0
        high = 0

        if ((flags & (MONO_FLAG | FALSE_STEREO)) == 0) : # if not mono
            entidx = 1 - entidx

        if ((w.c[0].median[0] & ~1) == 0 and w.holding_zero == 0 and w.holding_one == 0
            and (w.c[1].median[0] & ~1) == 0) :

            mask = 0
            cbits = 0

            if (w.zeros_acc > 0) :
                w.zeros_acc = w.zeros_acc - 1

                if (w.zeros_acc > 0) :
                    c[entidx].slow_level -= (c[entidx].slow_level + SLO) >> SLS;
                    buffer[buffer_counter] = 0;
                    buffer_counter = buffer_counter + 1
                    continue;
            else :
                # section called by mono code
                cbits = 0;
                bs = getbit(bs);

                while (cbits < 33 and bs.bitval > 0) :
                    cbits = cbits + 1
                    bs = getbit(bs)

                if (cbits == 33) :
                    break;

                if (cbits < 2) :
                    w.zeros_acc = cbits
                else :

                    cbits = cbits - 1
                    
                    mask = 1
                    w.zeros_acc = 0
                    
                    while cbits > 0 :
                        bs = getbit(bs);

                        if (bs.bitval > 0) :
                            w.zeros_acc |= mask;
                        mask <<= 1
                        cbits = cbits - 1

                    w.zeros_acc |= mask;

                if (w.zeros_acc > 0) :
                    c[entidx].slow_level -= (c[entidx].slow_level + SLO) >> SLS;
                    w.c[0].median[0] = 0;
                    w.c[0].median[1] = 0;
                    w.c[0].median[2] = 0;
                    w.c[1].median[0] = 0;
                    w.c[1].median[1] = 0;
                    w.c[1].median[2] = 0;

                    buffer[buffer_counter] = 0;
                    buffer_counter = buffer_counter + 1
                    continue;

        if (w.holding_zero > 0) :
            ones_count = w.holding_zero = 0;
        else :
            next8 = 0
            uns_buf = 0

            if (bs.bc < 8) :

                bs.ptr = bs.ptr + 1
                bs.buf_index = bs.buf_index + 1

                if (bs.ptr == bs.end) :
                    bs = bs_read(bs);

                uns_buf = ord(bs.buf[bs.buf_index]) & 0xff

                bs.sr = bs.sr | (uns_buf << bs.bc); # values in buffer must be unsigned

                next8 =  bs.sr & 0xff

                bs.bc += 8;
            else :
                next8 =  bs.sr & 0xff

            if (next8 == 0xff) :

                bs.bc -= 8;
                bs.sr >>= 8;

                ones_count = 8;
                bs = getbit(bs);

                while (ones_count < (LIMIT_ONES + 1) and bs.bitval > 0) :
                    ones_count = ones_count + 1
                    bs = getbit(bs)
                
                if (ones_count == (LIMIT_ONES + 1)) :
                    break;

                if (ones_count == LIMIT_ONES) :
                    mask = 0

                    cbits = 0;
                    bs = getbit(bs)

                    while (cbits < 33 and bs.bitval > 0) :
                        cbits = cbits + 1
                        bs = getbit(bs)
                    
                    if (cbits == 33) :
                        break;

                    if (cbits < 2) :
                        ones_count = cbits;
                    else :
                        mask = 1
                        ones_count = 0
                        
                        # We decrement cbits before entering while condition. This is to reflect the preincrement that is used
                        # in the Java version of the code

                        cbits = cbits - 1
                        
                        while cbits > 0 :
                            bs = getbit(bs)

                            if (bs.bitval > 0) :
                                ones_count |= mask

                            mask <<= 1
                            cbits = cbits - 1

                        
                        ones_count |= mask;

                    ones_count += LIMIT_ONES;

            else :
                ones_count = ones_count_table[next8]
                bs.bc -= ones_count + 1;
                bs.sr = bs.sr >> ones_count + 1; # needs to be unsigned

            if (w.holding_one > 0) :
                w.holding_one = ones_count & 1;
                ones_count = (ones_count >> 1) + 1;
            else :
                w.holding_one = ones_count & 1;
                ones_count >>= 1;

            w.holding_zero =  (~w.holding_one & 1);

        if ((flags & HYBRID_FLAG) > 0
            and ((flags & (MONO_FLAG | FALSE_STEREO)) > 0 or (csamples & 1) == 0)) :
            w = update_error_limit(w, flags);

        if (ones_count == 0) :
            low = 0;
            high = (((c[entidx].median[0]) >> 4) + 1) - 1;
            c[entidx].median[0] -= (((c[entidx].median[0] + (DIV0 - 2)) / DIV0) * 2);
        else :
            low = (((c[entidx].median[0]) >> 4) + 1);

            c[entidx].median[0] += ((c[entidx].median[0] + DIV0) / DIV0) * 5;

            if (ones_count == 1) :
                high = low + (((c[entidx].median[1]) >> 4) + 1) - 1;
                c[entidx].median[1] -= ((c[entidx].median[1] + (DIV1 - 2)) / DIV1) * 2;
            else :
                low += (((c[entidx].median[1]) >> 4) + 1);
                c[entidx].median[1] += ((c[entidx].median[1] + DIV1) / DIV1) * 5;

                if (ones_count == 2) :
                    high = low + (((c[entidx].median[2]) >> 4) + 1) - 1;
                    c[entidx].median[2] -= ((c[entidx].median[2] + (DIV2 - 2)) / DIV2) * 2;
                else :
                    low += (ones_count - 2) * (((c[entidx].median[2]) >> 4) + 1);
                    high = low + (((c[entidx].median[2]) >> 4) + 1) - 1;
                    c[entidx].median[2] += ((c[entidx].median[2] + DIV2) / DIV2) * 5;

        mid = (high + low + 1) >> 1;

        if (c[entidx].error_limit == 0) :
            mid = read_code(bs, high - low)

            mid = mid + low
        else :
            while (high - low > c[entidx].error_limit) :

                bs = getbit(bs)

                if (bs.bitval > 0) :
                    low = mid
                    mid = (high + (low) + 1) >> 1
                else :
                    high = mid -1
                    mid = ((high) + low + 1) >> 1


        bs = getbit(bs);

        if (bs.bitval > 0) :
            buffer[buffer_counter] = ~mid
        else :
            buffer[buffer_counter] = mid

        buffer_counter = buffer_counter + 1

        if ((flags & HYBRID_BITRATE) > 0) :
            c[entidx].slow_level = c[entidx].slow_level - ((c[entidx].slow_level + SLO) >> SLS) + mylog2(mid)

    w.c = c
    csamples =  csamples + 1

    if ((flags & (MONO_FLAG | FALSE_STEREO)) != 0) :
        return csamples;
    else :
        return (csamples / 2);


def count_bits(av) :
    if (av < (1 << 8)) :
        return nbits_table[av]
    else :
        if (av < (1 << 16)) :
            return nbits_table[ (av >> 8)] + 8
        else :
            if (av < (1 << 24)) :
                return nbits_table[ (av >> 16)] + 16
            else :
                return nbits_table[ (av >> 24)] + 24


# Read a single unsigned value from the specified bitstream with a value
# from 0 to maxcode. If there are exactly a power of two number of possible
# codes then this will read a fixed number of bits; otherwise it reads the
# minimum number of bits and then determines whether another bit is needed
# to define the code.

def read_code( bs, maxcode) :
    bitcount = count_bits(maxcode);
    extras = (1L << bitcount) - maxcode - 1
    code = 0

    if (bitcount == 0) :
        return (0)

    code = getbits(bitcount - 1, bs)
    
    code &= (1L << (bitcount - 1)) - 1

    if (code >= extras) :
        code = (code + code) - extras

        bs = getbit(bs)

        if (bs.bitval > 0) :
            code = code + 1

    return (code)


# The concept of a base 2 logarithm is used in many parts of WavPack. It is
# a way of sufficiently accurately representing 32-bit signed and unsigned
# values storing only 16 bits (actually fewer). It is also used in the hybrid
# mode for quickly comparing the relative magnitude of large values (i.e.
# division) and providing smooth exponentials using only addition.

# These are not strict logarithms in that they become linear around zero and
# can therefore represent both zero and negative values. They have 8 bits
# of precision and in "roundtrip" conversions the total error never exceeds 1
# part in 225 except for the cases of +/-115 and +/-195 (which error by 1).


# This function returns the log2 for the specified 32-bit unsigned value.
# The maximum value allowed is about 0xff800000 and returns 8447.

def mylog2(avalue) :
    dbits = 0

    avalue = avalue + (avalue >> 9)
    if (avalue  < (1 << 8)) :
        dbits = nbits_table[avalue]
        return (dbits << 8) + log2_table[(avalue << (9 - dbits)) & 0xff]
    else :
        if (avalue < (1L << 16)) :
            dbits = nbits_table[(avalue >> 8)] + 8

        elif (avalue < (1L << 24)) :
            dbits = nbits_table[(avalue >> 16)] + 16

        else :
            dbits = nbits_table[(avalue >> 24)] + 24

        return (dbits << 8) + log2_table[(avalue >> (dbits - 9)) & 0xff]


# This function returns the log2 for the specified 32-bit signed value.
# All input values are valid and the return values are in the range of
# +/- 8192.

def log2s(value) :
    if (value < 0) :
        return -mylog2(-value)
    else :
        return mylog2(value)


# This function returns the original integer represented by the supplied
# logarithm (at least within the provided accuracy). The log is signed,
# but since a full 32-bit value is returned this can be used for unsigned
# conversions as well (i.e. the input range is -8192 to +8447).

def exp2s(log) :
    value = 0

    if (log < 0) :
        return -exp2s(-log)
    
    value = exp2_table[log & 0xff] | 0x100

    log = log >> 8
    if ( log <= 9) :
        return (value >> (9 - log)) & 0xffffffff
    else :
        return value << (log - 9)


# These two functions convert internal weights (which are normally +/-1024)
# to and from an 8-bit signed character version for storage in metadata. The
# weights are clipped here in the case that they are outside that range.

def restore_weight(weight) :
    result = 0

    result = weight << 3
    
    if ( result > 0) :
        result += (result + 64) >> 7

    return result


