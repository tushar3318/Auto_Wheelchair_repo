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
import errno
import os
import argparse
import soundfile as sf
import numpy as np
import librosa

THRESHOLD = 30  # threshold to detect the beginning on an utterance


def resample(folder_in, folder_out, sr=16384):
    """
    Detects utterances in the input audio files and creates 1-sec 16khz
    mono .wav files as needed for KWS dataset
    """

    # create output folder
    try:
        os.mkdir(folder_out)
    except OSError as e:
        if e.errno == errno.EEXIST:
            pass
        else:
            raise
        print(f'{folder_out} already exists. overwrite!')

    for (dirpath, _, filenames) in os.walk(folder_in):
        for filename in sorted(filenames):

            file_cnt = 0
            i = 0
            if filename.endswith('.wav') or filename.endswith('.ogg'):
                fname = os.path.join(dirpath, filename)
                data, samplerate = librosa.load(fname, sr=sr)
                print(f'\rProcessing {fname}, sample rate={samplerate}', end="   ")
                mx = np.amax(abs(data))
                data = data/mx
                chunk_start = 0
                segment_len = 98*128

                while True:
                    if chunk_start + segment_len > len(data):
                        break

                    chunk = data[chunk_start: chunk_start+128]
                    avg = 1000*np.average(abs(chunk))

                    # visualize:
                    # bars = "=" * int(100 * avg/100)
                    # peak = avg * 100
                    # print("%04d %05d %s" % (i, peak, bars))

                    i += 128
                    if avg > THRESHOLD and chunk_start >= 30*128:
                        frame = data[chunk_start - 30*128:chunk_start + 98*128]
                        outfile = os.path.join(folder_out, filename[:-4] + str(file_cnt) + ".wav")
                        sf.write(outfile, frame, sr)
                        file_cnt += 1
                        chunk_start += 98*128
                    else:
                        chunk_start += 128
            else:
                continue
    print('\r')


def command_parser():
    """
    Return the argument parser
    """
    parser = argparse.ArgumentParser(description='Audio recorder command parser')
    parser.add_argument('-i', '--input', type=str, default='InputFolder', required=False,
                        help='input folder with audio files')

    parser.add_argument('-o', '--output', type=str, required=False, default='OutputWav',
                        help='output folder for segmented and resampled audio files')
    return parser.parse_args()


if __name__ == "__main__":
    command = command_parser()
    resample(command.input, command.output)
