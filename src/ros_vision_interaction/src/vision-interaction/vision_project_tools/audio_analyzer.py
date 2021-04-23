#!/usr/bin/env python3.8
import glob
import logging
import os
import shutil

from pydub import AudioSegment

logging.basicConfig(level=logging.INFO)


class AudioAnalyzer:

    def __init__(
            self,
            upload_directory,
            copy_directory,
            file_prefix,
            new_file_name,
            extension="wav",
            silence_threshold=-20,
            chunk_size=10
    ):
        self._upload_directory = upload_directory
        self._copy_directory = copy_directory
        self._file_prefix = file_prefix
        self._new_file_name = new_file_name
        self._extension = extension
        self._new_audio_file_path = os.path.join(
            self._copy_directory,
            "{new_file_name}.{ext}".format(new_file_name=self._new_file_name, ext=self._extension)
        )
        if not os.path.exists(self._copy_directory):
            os.makedirs(self._copy_directory)
        self._silence_threshold = silence_threshold
        self._chunk_size = chunk_size

    def get_length_of_trimmed_audio(self):
        audio_file = self.find_audio_file()
        self.copy_evaluation_file(audio_file, self._new_audio_file_path)
        audio_segment = self.get_trimmed_audio_segment_from_file(self._new_audio_file_path, self._chunk_size)
        self.save_cropped_audio(audio_segment, self._new_file_name)
        return len(audio_segment)

    def get_trimmed_audio_segment_from_file(self, file_name, chunk_size):
        if not file_name.endswith(self._extension):
            raise TypeError("Must be a {} file".format(self._extension))
        audio = AudioSegment.from_file(file_name, format=self._extension)
        logging.info("Audio length: {}".format(len(audio)))
        leading_silence_duration = self._get_leading_silence_duration(audio, chunk_size)
        ending_silence_duration = self._get_leading_silence_duration(audio.reverse(), chunk_size)
        full_audio_length = len(audio)
        trimmed_audio = audio[leading_silence_duration:full_audio_length - ending_silence_duration]
        return trimmed_audio

    def _get_leading_silence_duration(self, audio, chunk_size):
        ms_to_trim = 0
        if chunk_size <= 0:
            raise ValueError("Audio chunk size must be greater than 0")

        while audio[ms_to_trim:ms_to_trim + chunk_size].dBFS < self._silence_threshold:
            ms_to_trim += chunk_size

        return ms_to_trim

    def find_audio_file(self):
        list_of_audio_files = glob.glob(os.path.join(self._upload_directory, "*." + self._extension))
        for file in list_of_audio_files:
            parts = file.split("\\")
            file_name = parts[len(parts)-1]
            if file_name.startswith(self._file_prefix):
                return file

    def copy_evaluation_file(self, file, dest):
        shutil.copy(file, dest)

    def save_cropped_audio(self, audio_segment, new_file_name):
        new_file_path = os.path.join(self._copy_directory, new_file_name)
        audio_segment.export(new_file_path, format("wav"))
