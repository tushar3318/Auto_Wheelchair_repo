###############################################################################
 #
 # Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by
 # Analog Devices, Inc.),
 # Copyright (C) 2023-2024 Analog Devices, Inc.
 #
 # Licensed under the Apache License, Version 2.0 (the "License");
 # you may not use this file except in compliance with the License.
 # You may obtain a copy of the License at
 #
 #     http://www.apache.org/licenses/LICENSE-2.0
 #
 # Unless required by applicable law or agreed to in writing, software
 # distributed under the License is distributed on an "AS IS" BASIS,
 # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 # See the License for the specific language governing permissions and
 # limitations under the License.
 #
 ##############################################################################
import os
import argparse
import soundfile as sf
import sounddevice as sd
import numpy as np
import matplotlib.pyplot as plt

def convertAll(topDir):
    """
    Walk directories from the root and convert all sample audio file to wave (.wav) files
    - skips if the file has already been converted.
    - skips if the file size is not exactly 16384.
    - returns number of files converted.
    """
    numConverted = 0
    for root, dirs, files in os.walk(topDir):
        for filename in files:
            input = os.path.join(root, filename) 
            if os.path.splitext(input)[-1] == '' and os.path.getsize(input) == 16384:
                output = input + ".wav"
            if not os.path.isfile(output):
                data = np.fromfile(input, dtype = 'int8')
                sf.write(output, 100 * data.astype('int16'), len(data))
                numConverted += 1
    return numConverted

def readbin(input, output):

    data = np.fromfile(input, dtype = 'int8')
    sd.play(data, 16384)
    print (data)
    plt.plot(data)
    plt.show()

    # save as .np
    np.save(output[:-4], data)

    sf.write(output, 100 * data.astype('int16'), len(data))

def command_parser():
    """
    Return the argument parser
    """
    parser = argparse.ArgumentParser(description='Convert bin to audio wav')
    parser.add_argument('-i', '--input', type=str, required = False,
                        help='input bin file')

    parser.add_argument('-o', '--output', type=str, required = False, default = 'out.wav',
                        help='output wav')
    
    parser.add_argument('-a', '--all', required = False, action="store_true",
                        help='walk all directories and convert all files')
    
    parser.add_argument('-d', '--directory', type = str, required = False, default = ".",
                        help='top directory')
    return parser.parse_args()

if __name__ == "__main__":
    command = command_parser()
    if command.all is False:
        readbin(command.input, command.output)
    else:
        print(f"{convertAll(command.directory)} files are converted!")