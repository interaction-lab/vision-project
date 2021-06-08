#!/usr/bin/env python3.8
import collections
import contextlib
import logging
import speech_recognition
import wave
import webrtcvad

from pydub import AudioSegment

logging.basicConfig(level=logging.INFO)


class Frame(object):
    """Represents a "frame" of audio data."""

    def __init__(self, bytes, timestamp, duration):
        self.bytes = bytes
        self.timestamp = timestamp
        self.duration = duration


class ReadingEvaluator:
    """Finds and """

    def __init__(
            self,
            silence_threshold=-35,
            sample_rate=16000,
            chunk_size=10,
    ):
        self._silence_threshold = silence_threshold
        self._sample_rate = sample_rate
        self._chunk_size = chunk_size

        self._sr = speech_recognition.Recognizer()
        self._vad = webrtcvad.Vad()
        self._vad.set_mode(3)

    def get_total_speaking_time(self, audio_file_path):
        total_speaking_time = 0.0
        audio, sample_rate = self.read_wav(audio_file_path)
        frame_duration_ms = 30
        padding_duration_ms = 300
        frames = list(self.generate_frames(frame_duration_ms, audio, sample_rate))
        voiced_segments = self.get_voiced_audio_from_frames(
            sample_rate,
            frame_duration_ms,
            padding_duration_ms,
            frames
        )
        for segment in voiced_segments:
            audio_segment = AudioSegment(segment, sample_width=2, frame_rate=sample_rate, channels=1)
            total_speaking_time += audio_segment.duration_seconds
        return total_speaking_time

    def read_wav(self, path):
        """Reads a .wav file.

        Takes the path, and returns (PCM audio data, sample rate).
        """
        with contextlib.closing(wave.open(path, 'rb')) as wf:
            num_channels = wf.getnchannels()
            if num_channels != 1:
                raise ValueError("Invalid number of channels")
            sample_width = wf.getsampwidth()
            if sample_width != 2:
                raise ValueError("Invalid sample width")
            sample_rate = wf.getframerate()
            if sample_rate not in (8000, 16000, 32000, 48000):
                raise ValueError("Invalid sample rate")
            pcm_data = wf.readframes(wf.getnframes())
            return pcm_data, sample_rate

    def generate_frames(self, frame_duration_ms, audio, sample_rate):
        """Generates audio frames from PCM audio data.

        Takes the desired frame duration in milliseconds, the PCM data, and
        the sample rate.

        Yields Frames of the requested duration.
        """
        n = int(sample_rate * (frame_duration_ms / 1000.0) * 2)
        offset = 0
        timestamp = 0.0
        duration = (float(n) / sample_rate) / 2.0
        while offset + n < len(audio):
            yield Frame(audio[offset:offset + n], timestamp, duration)
            timestamp += duration
            offset += n

    def get_voiced_audio_from_frames(
            self,
            sample_rate,
            frame_duration_ms,
            padding_duration_ms,
            frames
    ):
        """Filters out non-voices audio frames and yields the resulting voices audio data in bytes."""
        num_padding_frames = int(padding_duration_ms / frame_duration_ms)
        ring_buffer = collections.deque(maxlen=num_padding_frames)
        triggered = False

        voiced_frames = []
        for frame in frames:
            is_speech = self._vad.is_speech(frame.bytes, sample_rate)

            if not triggered:
                ring_buffer.append((frame, is_speech))
                num_voiced = len([f for f, speech in ring_buffer if speech])
                if num_voiced > 0.9 * ring_buffer.maxlen:
                    triggered = True
                    for f, s in ring_buffer:
                        voiced_frames.append(f)
                    ring_buffer.clear()
            else:
                voiced_frames.append(frame)
                ring_buffer.append((frame, is_speech))
                num_unvoiced = len([f for f, speech in ring_buffer if not speech])
                if num_unvoiced > 0.9 * ring_buffer.maxlen:
                    triggered = False
                    yield b''.join([f.bytes for f in voiced_frames])
                    ring_buffer.clear()
                    voiced_frames = []
        # Yield any leftover frames
        if voiced_frames:
            yield b''.join([f.bytes for f in voiced_frames])
