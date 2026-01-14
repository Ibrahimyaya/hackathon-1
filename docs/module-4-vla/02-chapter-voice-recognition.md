# Chapter 1: Voice-to-Action Pipeline with Whisper

## Overview

This chapter teaches you how to capture voice commands from users and convert them into structured, machine-readable format using OpenAI's Whisper speech-to-text engine.

## Learning Objectives

By the end of this chapter, you will:
- Understand Whisper's architecture and capabilities
- Set up Whisper locally or via API
- Build real-time voice input pipelines
- Handle transcription errors gracefully
- Integrate voice input with ROS 2 systems

## Table of Contents

1. [What is Whisper?](#what-is-whisper)
2. [Setup & Installation](#setup--installation)
3. [Basic Voice Transcription](#basic-voice-transcription)
4. [Real-time Voice Pipeline](#real-time-voice-pipeline)
5. [Error Handling & Retry Logic](#error-handling--retry-logic)
6. [Integration with ROS 2](#integration-with-ros-2)

## What is Whisper?

Whisper is OpenAI's robust speech recognition model that can transcribe audio in multiple languages with high accuracy.

### Key Features

- **Multilingual**: Supports 99+ languages
- **Robust**: Handles accents, background noise, technical language
- **Flexible**: Available as API or local open-source model
- **Efficient**: Runs on CPU with modest resource requirements

### Trade-offs: Local vs API

| Aspect | Local Whisper | Whisper API |
|--------|---------------|------------|
| **Cost** | Free (one-time download) | $0.02 per minute audio |
| **Latency** | 100-500ms (depends on model size) | 1-5s (network) |
| **Privacy** | Data stays on device | Data sent to OpenAI |
| **Accuracy** | Excellent | Excellent |
| **Setup** | Complex | Simple (1 API key) |

## Setup & Installation

### Option 1: Using OpenAI Whisper API

```bash
pip install openai
```

```python
import openai

openai.api_key = "your-api-key"

with open("audio.mp3", "rb") as audio_file:
    transcript = openai.Audio.transcribe("whisper-1", audio_file)

print(transcript["text"])
```

### Option 2: Using Local Whisper Model

```bash
pip install openai-whisper
```

```python
import whisper

model = whisper.load_model("base")
result = model.transcribe("audio.mp3")
print(result["text"])
```

## Basic Voice Transcription

### Microphone Input Example

```python
import whisper
import pyaudio
import numpy as np

def record_audio(duration=5):
    """Record audio from microphone for specified duration"""
    CHUNK = 1024
    FORMAT = pyaudio.paFloat32
    CHANNELS = 1
    RATE = 16000

    p = pyaudio.PyAudio()
    stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE,
                    input=True, frames_per_buffer=CHUNK)

    frames = []
    for _ in range(0, int(RATE / CHUNK * duration)):
        data = stream.read(CHUNK)
        frames.append(np.frombuffer(data, dtype=np.float32))

    stream.stop_stream()
    stream.close()
    p.terminate()

    return np.concatenate(frames)

def transcribe_microphone():
    """Record and transcribe audio"""
    print("Recording... speak now")
    audio = record_audio(duration=5)

    model = whisper.load_model("base")
    result = model.transcribe_np(audio, fp16=False)

    print(f"You said: {result['text']}")
    return result['text']
```

## Real-time Voice Pipeline

### Streaming Recognition

```python
import threading
from collections import deque

class RealTimeVoicePipeline:
    def __init__(self, callback):
        self.callback = callback
        self.buffer = deque(maxlen=3)  # Last 3 seconds
        self.running = False

    def start(self):
        """Start capturing and transcribing"""
        self.running = True
        self.capture_thread = threading.Thread(target=self._capture_loop)
        self.transcribe_thread = threading.Thread(target=self._transcribe_loop)
        self.capture_thread.start()
        self.transcribe_thread.start()

    def _capture_loop(self):
        """Continuously capture audio"""
        p = pyaudio.PyAudio()
        stream = p.open(format=pyaudio.paFloat32, channels=1,
                       rate=16000, input=True, frames_per_buffer=1024)

        while self.running:
            data = stream.read(1024)
            self.buffer.append(np.frombuffer(data, dtype=np.float32))

        stream.stop_stream()
        stream.close()
        p.terminate()

    def _transcribe_loop(self):
        """Transcribe buffered audio"""
        model = whisper.load_model("tiny")  # Faster model for streaming

        while self.running:
            if len(self.buffer) == self.buffer.maxlen:
                audio = np.concatenate(list(self.buffer))
                result = model.transcribe_np(audio, fp16=False)
                self.callback(result['text'])
```

## Error Handling & Retry Logic

### Robust Transcription with Fallbacks

```python
import logging

logger = logging.getLogger(__name__)

def robust_transcribe(audio_file, max_retries=3):
    """Transcribe with retry logic and fallbacks"""

    for attempt in range(max_retries):
        try:
            model = whisper.load_model("base")
            result = model.transcribe(audio_file)
            return result["text"]

        except Exception as e:
            logger.warning(f"Attempt {attempt + 1} failed: {e}")

            if attempt < max_retries - 1:
                # Retry with smaller model
                try:
                    model = whisper.load_model("tiny")
                    result = model.transcribe(audio_file)
                    return result["text"]
                except Exception as e2:
                    logger.error(f"Fallback also failed: {e2}")
                    continue
            else:
                raise
```

## Integration with ROS 2

### Publishing Voice Commands as ROS 2 Topics

```python
import rclpy
from std_msgs.msg import String

class VoiceCommandPublisher(rclpy.node.Node):
    def __init__(self):
        super().__init__('voice_command_publisher')
        self.publisher = self.create_publisher(String, 'voice_command', 10)

    def publish_voice_command(self, text):
        """Publish transcribed voice as ROS 2 message"""
        msg = String()
        msg.data = text
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {text}')

def main():
    rclpy.init()
    publisher = VoiceCommandPublisher()

    while True:
        text = transcribe_microphone()
        publisher.publish_voice_command(text)

if __name__ == '__main__':
    main()
```

## Summary

You now understand how to:
- Use Whisper for robust speech-to-text
- Choose between local and API approaches
- Handle errors and retries
- Integrate voice input into ROS 2 systems

## Next Steps

→ Return to [Module 4 Introduction](intro.md)
