# Module 4 Quick-Start Guide

**Goal**: Get a working VLA example running in 30 minutes

---

## Prerequisites Checklist

Before starting, verify you have:

- [ ] Ubuntu 22.04 LTS with ROS 2 Humble or Jazzy installed
- [ ] Python 3.10+ (`python3 --version`)
- [ ] NVIDIA GPU with 8GB+ VRAM (`nvidia-smi`)
- [ ] OpenAI API key (https://platform.openai.com/api-keys)
- [ ] Internet connection for API calls

---

## 5-Minute Setup

### Step 1: Install Python Dependencies

```bash
pip install openai pydub pyaudio pyttsx3 pyyaml
```

**Expected Output**:
```
Successfully installed openai pydub pyaudio pyttsx3 pyyaml
```

### Step 2: Set Environment Variables

```bash
export OPENAI_API_KEY="sk-your-key-here"
export OPENAI_ORG_ID="org-your-org-id"  # Optional
```

**Verify**:
```bash
echo $OPENAI_API_KEY
# Output: sk-...
```

### Step 3: Initialize ROS 2

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # Or your preferred DDS
```

---

## Part 1: Voice-to-Action (Chapter 1) - 10 minutes

### Goal
Capture a voice command, transcribe it with Whisper, output the text.

### Code Example

Create `chapter1_quickstart.py`:

```python
#!/usr/bin/env python3
"""
Module 4 Chapter 1 Quick-Start: Voice-to-Action with Whisper
"""

import os
import json
from openai import OpenAI
from pathlib import Path

# Initialize OpenAI client
api_key = os.getenv("OPENAI_API_KEY")
if not api_key:
    raise ValueError("OPENAI_API_KEY environment variable not set")

client = OpenAI(api_key=api_key)

def transcribe_audio(audio_file_path: str) -> str:
    """
    Transcribe audio file using Whisper API.

    Args:
        audio_file_path: Path to audio file (mp3, wav, m4a, flac, ogg, webm, etc.)

    Returns:
        Transcribed text
    """
    if not Path(audio_file_path).exists():
        raise FileNotFoundError(f"Audio file not found: {audio_file_path}")

    with open(audio_file_path, "rb") as audio_file:
        transcript = client.audio.transcriptions.create(
            model="whisper-1",
            file=audio_file,
            language="en"
        )

    return transcript.text

if __name__ == "__main__":
    # Example: Transcribe a sample audio file
    # For demo, we'll create a simple test

    print("=" * 60)
    print("Module 4 Chapter 1: Voice-to-Action Quick-Start")
    print("=" * 60)

    # Test with a sample audio file (you provide this)
    sample_audio = "sample_voice.mp3"  # Provide your own or create one

    if Path(sample_audio).exists():
        print(f"\nTranscribing: {sample_audio}")
        try:
            text = transcribe_audio(sample_audio)
            print(f"\n✅ Transcription Result:")
            print(f"   {text}")
        except Exception as e:
            print(f"❌ Error: {e}")
    else:
        print(f"\n⚠️  Sample audio not found: {sample_audio}")
        print("\nTo test, provide an audio file and set sample_audio variable")
        print("\nExample usage:")
        print("  from openai import OpenAI")
        print("  client = OpenAI()")
        print("  with open('your_audio.mp3', 'rb') as f:")
        print("      transcript = client.audio.transcriptions.create(")
        print("          model='whisper-1',")
        print("          file=f")
        print("      )")
        print("      print(transcript.text)")
```

### Run It

```bash
# Download or provide a sample audio file (e.g., "sample_voice.mp3")
# Then run:
python chapter1_quickstart.py
```

**Expected Output**:
```
============================================================
Module 4 Chapter 1: Voice-to-Action Quick-Start
============================================================

Transcribing: sample_voice.mp3

✅ Transcription Result:
   Hello, pick up the blue ball
```

---

## Part 2: LLM Planning (Chapter 2) - 10 minutes

### Goal
Take a voice command and decompose it into a task plan using GPT-4.

### Code Example

Create `chapter2_quickstart.py`:

```python
#!/usr/bin/env python3
"""
Module 4 Chapter 2 Quick-Start: LLM Cognitive Planning with GPT-4
"""

import os
import json
from openai import OpenAI

# Initialize OpenAI client
api_key = os.getenv("OPENAI_API_KEY")
if not api_key:
    raise ValueError("OPENAI_API_KEY environment variable not set")

client = OpenAI(api_key=api_key)

# System prompt for task planning
PLANNER_PROMPT = """
You are a task planning system for a humanoid robot.

ROBOT CAPABILITIES:
- Max reach: 1.5 meters
- Max grip force: 50 Newtons
- Max object weight: 2 kg
- Workspace: 5m x 5m floor area
- Speed: 0.5 m/s walking, 2 m/s grasping

ROBOT CONSTRAINTS:
- No water interaction
- No climbing
- No electrical hazards
- Safety zone around humans: 0.5m

Your task:
1. Understand the natural language command
2. Decompose into executable sub-goals
3. Create action sequence (navigate, grasp, manipulate, place)
4. Validate feasibility against robot capabilities
5. Return JSON task plan

Return format (MUST be valid JSON):
{
    "goal": "main objective",
    "sub_goals": ["goal1", "goal2", ...],
    "action_sequence": [
        {"action": "navigate", "target": "location"},
        {"action": "grasp", "object": "name"},
        ...
    ],
    "feasibility": "yes" or "no",
    "confidence": 0.95,
    "safety_notes": "any risks"
}
"""

def plan_task(voice_command: str) -> dict:
    """
    Decompose voice command into task plan using GPT-4.

    Args:
        voice_command: Natural language command (e.g., "pick up the blue ball")

    Returns:
        Task plan as dictionary
    """
    response = client.chat.completions.create(
        model="gpt-4",
        messages=[
            {"role": "system", "content": PLANNER_PROMPT},
            {"role": "user", "content": voice_command}
        ],
        temperature=0.3,
        max_tokens=500
    )

    # Extract JSON from response
    plan_text = response.choices[0].message.content

    try:
        plan = json.loads(plan_text)
    except json.JSONDecodeError:
        # Fallback if response isn't valid JSON
        plan = {
            "goal": voice_command,
            "sub_goals": [],
            "action_sequence": [],
            "feasibility": "unknown",
            "confidence": 0.0,
            "safety_notes": "Could not parse plan"
        }

    return plan

if __name__ == "__main__":
    print("=" * 60)
    print("Module 4 Chapter 2: LLM Planning Quick-Start")
    print("=" * 60)

    # Example voice commands
    test_commands = [
        "Pick up the blue ball",
        "Move the cup from the table to the sink",
        "Navigate to the kitchen and prepare the counter"
    ]

    for command in test_commands:
        print(f"\n📌 Command: {command}")
        try:
            plan = plan_task(command)
            print(f"✅ Task Plan Generated:")
            print(f"   Goal: {plan.get('goal', 'N/A')}")
            print(f"   Feasibility: {plan.get('feasibility', 'unknown')}")
            print(f"   Confidence: {plan.get('confidence', 0.0):.2f}")
            print(f"   Actions: {len(plan.get('action_sequence', []))} steps")

            if plan.get('safety_notes'):
                print(f"   ⚠️  Safety: {plan['safety_notes']}")

        except Exception as e:
            print(f"❌ Error: {e}")
```

### Run It

```bash
python chapter2_quickstart.py
```

**Expected Output**:
```
============================================================
Module 4 Chapter 2: LLM Planning Quick-Start
============================================================

📌 Command: Pick up the blue ball
✅ Task Plan Generated:
   Goal: Pick up the blue ball
   Feasibility: yes
   Confidence: 0.92
   Actions: 4 steps

📌 Command: Move the cup from the table to the sink
✅ Task Plan Generated:
   Goal: Move the cup from the table to the sink
   Feasibility: yes
   Confidence: 0.88
   Actions: 5 steps

📌 Command: Navigate to the kitchen and prepare the counter
✅ Task Plan Generated:
   Goal: Navigate to the kitchen and prepare the counter
   Feasibility: yes
   Confidence: 0.85
   Actions: 6 steps
```

---

## Part 3: ROS 2 Action Execution (Chapter 3) - 5 minutes

### Goal
Set up a simple ROS 2 action client for executing navigation commands.

### Prerequisites
```bash
# Install ROS 2 action examples (if not already installed)
sudo apt install ros-humble-demo-nodes-cpp ros-humble-demo-nodes-py
```

### Code Example

Create `chapter3_quickstart.py`:

```python
#!/usr/bin/env python3
"""
Module 4 Chapter 3 Quick-Start: ROS 2 Action Execution
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from example_interfaces.action import Fibonacci
import time

class SimpleActionClient(Node):
    def __init__(self):
        super().__init__('simple_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.sequence}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')

def main():
    rclpy.init()

    action_client = SimpleActionClient()

    print("=" * 60)
    print("Module 4 Chapter 3: ROS 2 Action Execution Quick-Start")
    print("=" * 60)

    print("\n📌 Setting up ROS 2 action client...")
    print("   Waiting for action server (fibonacci)...")

    action_client.send_goal(10)

    print("   Goal sent! Waiting for result...")

    # Spin and wait for result
    rclpy.spin(action_client)

    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Run It

```bash
# In Terminal 1: Start the action server
ros2 run demo_nodes_py fibonacci_action_server

# In Terminal 2: Run the client
python chapter3_quickstart.py
```

**Expected Output**:
```
============================================================
Module 4 Chapter 3: ROS 2 Action Execution Quick-Start
============================================================

📌 Setting up ROS 2 action client...
   Waiting for action server (fibonacci)...
   Goal sent! Waiting for result...
   Received feedback: [0, 1]
   Received feedback: [0, 1, 1]
   Received feedback: [0, 1, 1, 2]
   ...
   Result: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55]
```

---

## Part 4: Integration Test (Capstone) - 5 minutes

### Goal
Run a simple end-to-end test combining Whisper + GPT-4 + ROS 2.

### Code Example

Create `capstone_quickstart.py`:

```python
#!/usr/bin/env python3
"""
Module 4 Capstone Quick-Start: End-to-End VLA System
"""

import os
import json
from openai import OpenAI

# Initialize OpenAI client
api_key = os.getenv("OPENAI_API_KEY")
if not api_key:
    raise ValueError("OPENAI_API_KEY environment variable not set")

client = OpenAI(api_key=api_key)

PLANNER_PROMPT = """You are a task planner for a humanoid robot with capabilities: navigate, grasp, place.
Return ONLY valid JSON with keys: goal, sub_goals, action_sequence (list of dicts with 'action' and 'target' keys), feasibility (yes/no), confidence (0.0-1.0)."""

def voice_to_text(command: str) -> str:
    """Simulate voice-to-text (in real app, use Whisper)."""
    return command

def text_to_plan(text: str) -> dict:
    """Convert text to task plan using GPT-4."""
    response = client.chat.completions.create(
        model="gpt-4",
        messages=[
            {"role": "system", "content": PLANNER_PROMPT},
            {"role": "user", "content": text}
        ],
        temperature=0.3,
        max_tokens=300
    )

    try:
        return json.loads(response.choices[0].message.content)
    except:
        return {"goal": text, "feasibility": "unknown"}

def execute_plan(plan: dict) -> str:
    """Simulate plan execution (in real app, use ROS 2 actions)."""
    if plan.get("feasibility") == "yes":
        return "✅ Task executed successfully"
    else:
        return "❌ Task not feasible"

if __name__ == "__main__":
    print("=" * 60)
    print("Module 4 Capstone: End-to-End VLA Integration")
    print("=" * 60)

    # Test commands
    test_command = "Pick up the blue ball and place it on the table"

    print(f"\n🎤 Voice Input (simulated): {test_command}")

    # Step 1: Voice to text
    text = voice_to_text(test_command)
    print(f"📝 Transcribed Text: {text}")

    # Step 2: Text to plan
    print(f"\n🤖 Planning with GPT-4...")
    plan = text_to_plan(text)
    print(f"✅ Plan Generated:")
    print(f"   Goal: {plan.get('goal')}")
    print(f"   Feasibility: {plan.get('feasibility')}")
    print(f"   Confidence: {plan.get('confidence', 0.0):.2f}")

    # Step 3: Execute plan
    print(f"\n🚀 Executing Plan...")
    result = execute_plan(plan)
    print(f"   {result}")

    print(f"\n✨ VLA Pipeline Complete!")
```

### Run It

```bash
python capstone_quickstart.py
```

**Expected Output**:
```
============================================================
Module 4 Capstone: End-to-End VLA Integration
============================================================

🎤 Voice Input (simulated): Pick up the blue ball and place it on the table
📝 Transcribed Text: Pick up the blue ball and place it on the table

🤖 Planning with GPT-4...
✅ Plan Generated:
   Goal: Pick up the blue ball and place it on the table
   Feasibility: yes
   Confidence: 0.90

🚀 Executing Plan...
   ✅ Task executed successfully

✨ VLA Pipeline Complete!
```

---

## Troubleshooting

### OpenAI API Key Issues

```bash
# Verify key is set
echo $OPENAI_API_KEY

# If not set, add to .bashrc
echo 'export OPENAI_API_KEY="sk-your-key"' >> ~/.bashrc
source ~/.bashrc
```

### ROS 2 Issues

```bash
# Verify ROS 2 is sourced
echo $ROS_DISTRO
# Should output: humble or jazzy

# If not, source it
source /opt/ros/humble/setup.bash
```

### Python Package Issues

```bash
# Reinstall all dependencies
pip install --upgrade openai pydub pyaudio pyttsx3 pyyaml

# Verify installation
python3 -c "import openai; print(openai.__version__)"
```

---

## Next Steps

1. **Explore Chapter 1**: Read `docs/module-4-vla/02-chapter-voice-recognition.md`
2. **Try Chapter 1 Examples**: Run `code-examples/module-4/chapter-1/*.py`
3. **Review Data Model**: Check `specs/004-vla-integration/data-model.md`
4. **Read API Contracts**: Review `specs/004-vla-integration/contracts/`

---

## References

- **OpenAI Whisper**: https://platform.openai.com/docs/guides/speech-to-text
- **OpenAI GPT-4**: https://platform.openai.com/docs/guides/gpt
- **ROS 2 Actions**: https://docs.ros.org/en/humble/Concepts/Intermediate/Understanding-ROS-2-actions.html

---

**Quick-Start Version**: 1.0
**Last Updated**: 2026-01-08
**Estimated Time**: 30 minutes to run all 4 parts
