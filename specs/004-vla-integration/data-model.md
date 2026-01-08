# Data Model: Module 4 - Vision-Language-Action Integration

**Version**: 1.0
**Date**: 2026-01-08
**Stage**: Phase 1 Design (Complete)

---

## Overview

This document defines the core data entities and relationships for the VLA (Vision-Language-Action) system. These entities flow through the three-stage pipeline:

1. **Voice Command** → (Whisper) → Transcribed text
2. **Transcribed Text** → (GPT-4 Planning) → **Task Plan**
3. **Task Plan** → (ROS 2 Actionlib) → **Action Execution** → **Execution Feedback**

---

## Core Entities

### 1. Voice Command

**Purpose**: Capture natural language input from user speech

**Fields**:

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `command_id` | UUID | Yes | Unique identifier for command |
| `timestamp` | ISO 8601 datetime | Yes | When command was initiated |
| `audio_data` | Binary | Yes | Raw audio bytes or file path |
| `audio_format` | String | Yes | Format: wav, mp3, flac, ogg, webm |
| `duration_seconds` | Float | Yes | Length of audio in seconds |
| `sample_rate` | Integer | No | Sample rate (Hz), typically 16000 |
| `language` | String | No | Language code (e.g., "en", "es") |
| `user_id` | String | No | User who issued command |
| `context` | Object | No | Additional context (e.g., current location, objects in view) |

**Example**:

```json
{
  "command_id": "cmd_20260108_001",
  "timestamp": "2026-01-08T14:32:45Z",
  "audio_data": "audio_20260108_001.wav",
  "audio_format": "wav",
  "duration_seconds": 2.5,
  "sample_rate": 16000,
  "language": "en",
  "user_id": "user_alice",
  "context": {
    "location": "kitchen",
    "objects_visible": ["table", "chair", "cup"]
  }
}
```

**Lifecycle**:
1. Created: User speaks command
2. Processed: Audio captured and buffered
3. Transcribed: Passed to Whisper module
4. Completed: Transcription result stored

---

### 2. Transcribed Text

**Purpose**: Store the textual representation of voice command

**Fields**:

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `transcription_id` | UUID | Yes | Unique identifier for transcription |
| `command_id` | UUID | Yes | Reference to original voice command |
| `text` | String | Yes | Transcribed text (natural language) |
| `confidence_score` | Float | Yes | Whisper confidence 0.0-1.0 |
| `language` | String | Yes | Detected language code |
| `alternatives` | Array[String] | No | Alternative transcriptions (N-best) |
| `processing_time_ms` | Integer | Yes | Time to transcribe (milliseconds) |
| `timestamp` | ISO 8601 datetime | Yes | When transcription completed |
| `whisper_model` | String | Yes | Model used (e.g., "whisper-1", "small") |

**Example**:

```json
{
  "transcription_id": "trans_20260108_001",
  "command_id": "cmd_20260108_001",
  "text": "Pick up the blue ball",
  "confidence_score": 0.96,
  "language": "en",
  "alternatives": [
    "Pick up the blue ball",
    "Pick up the blue wall",
    "Pick up the blue all"
  ],
  "processing_time_ms": 485,
  "timestamp": "2026-01-08T14:32:47Z",
  "whisper_model": "whisper-1"
}
```

**Lifecycle**:
1. Created: Whisper processes audio
2. Stored: Transcription result saved
3. Referenced: Used in task planning
4. Archived: Retained for audit trail

---

### 3. Task Plan

**Purpose**: Structured breakdown of complex commands into executable actions

**Fields**:

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `plan_id` | UUID | Yes | Unique identifier for plan |
| `transcription_id` | UUID | Yes | Reference to transcribed text |
| `goal` | String | Yes | High-level objective (natural language) |
| `sub_goals` | Array[String] | No | Intermediate objectives |
| `action_sequence` | Array[Action] | Yes | Ordered list of ROS 2 actions |
| `feasibility` | Enum | Yes | "yes", "no", or "partial" |
| `feasibility_reason` | String | No | Explanation if not feasible |
| `confidence_score` | Float | Yes | LLM confidence in plan 0.0-1.0 |
| `estimated_duration_seconds` | Float | Yes | Expected execution time |
| `safety_notes` | String | No | Potential risks or constraints |
| `processing_time_ms` | Integer | Yes | Time to generate plan (ms) |
| `timestamp` | ISO 8601 datetime | Yes | When plan was generated |
| `llm_model` | String | Yes | Model used (e.g., "gpt-4") |
| `dependencies` | Array[String] | No | Action dependencies (e.g., "2 depends on 1") |

**Example**:

```json
{
  "plan_id": "plan_20260108_001",
  "transcription_id": "trans_20260108_001",
  "goal": "Pick up the blue ball",
  "sub_goals": [
    "Navigate to ball location",
    "Grasp the blue ball",
    "Bring to starting position"
  ],
  "action_sequence": [
    {
      "action_id": 1,
      "action": "navigate",
      "target_location": "ball_location",
      "parameters": {"x": 1.5, "y": 2.0, "z": 0.0}
    },
    {
      "action_id": 2,
      "action": "grasp",
      "object_id": "blue_ball",
      "grip_force": "medium",
      "depends_on": [1]
    },
    {
      "action_id": 3,
      "action": "navigate",
      "target_location": "initial_position",
      "parameters": {"x": 0.0, "y": 0.0, "z": 0.0},
      "depends_on": [2]
    }
  ],
  "feasibility": "yes",
  "feasibility_reason": null,
  "confidence_score": 0.92,
  "estimated_duration_seconds": 8.5,
  "safety_notes": "Clear path to ball verified. No humans in workspace.",
  "processing_time_ms": 1250,
  "timestamp": "2026-01-08T14:32:49Z",
  "llm_model": "gpt-4",
  "dependencies": [
    "action_2 depends on action_1",
    "action_3 depends on action_2"
  ]
}
```

**Lifecycle**:
1. Created: GPT-4 processes transcription
2. Validated: Feasibility checked
3. Approved: Human review (optional)
4. Executed: Passed to ROS 2 action executor
5. Archived: Retained with execution results

---

### 4. ROS 2 Action

**Purpose**: Individual executable command for robot hardware/simulation

**Fields**:

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `action_id` | Integer | Yes | Sequential ID in action sequence (1-N) |
| `action` | String | Yes | Action type (navigate, grasp, manipulate, place, release) |
| `parameters` | Object | Yes | Action-specific parameters |
| `timeout_seconds` | Float | Yes | Max time to execute action |
| `priority` | Enum | No | "high", "normal", "low" |
| `preemptible` | Boolean | No | Can this action be interrupted? |
| `depends_on` | Array[Integer] | No | IDs of preceding actions |
| `retry_count` | Integer | No | How many retries on failure (0-3) |
| `safety_enabled` | Boolean | Yes | Enable safety constraints? |

**Example (Navigate Action)**:

```json
{
  "action_id": 1,
  "action": "navigate",
  "parameters": {
    "target_location": "ball_location",
    "x": 1.5,
    "y": 2.0,
    "z": 0.0,
    "max_velocity": 0.5,
    "path_planner": "nav2"
  },
  "timeout_seconds": 30.0,
  "priority": "normal",
  "preemptible": true,
  "depends_on": [],
  "retry_count": 1,
  "safety_enabled": true
}
```

**Example (Grasp Action)**:

```json
{
  "action_id": 2,
  "action": "grasp",
  "parameters": {
    "object_id": "blue_ball",
    "grip_force_fraction": 0.7,
    "approach_velocity": 0.1,
    "grip_type": "precision"
  },
  "timeout_seconds": 15.0,
  "priority": "high",
  "preemptible": false,
  "depends_on": [1],
  "retry_count": 2,
  "safety_enabled": true
}
```

**Lifecycle**:
1. Created: Extracted from task plan
2. Queued: Added to action executor queue
3. Executing: ROS 2 action server processing
4. Feedback: Progress updates sent back
5. Complete: Result stored (success/failure)

---

### 5. Execution Feedback

**Purpose**: Real-time progress updates during action execution

**Fields**:

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `feedback_id` | UUID | Yes | Unique identifier for feedback event |
| `action_id` | Integer | Yes | ID of executing action |
| `plan_id` | UUID | Yes | Reference to parent task plan |
| `timestamp` | ISO 8601 datetime | Yes | When feedback generated |
| `status` | Enum | Yes | "planning", "executing", "paused", "success", "failure" |
| `progress_percentage` | Float | No | Progress 0.0-100.0 |
| `current_state` | Object | No | Current robot state (pose, gripper, etc.) |
| `message` | String | No | Human-readable status message |
| `error_code` | String | No | Error code if status is "failure" |
| `error_message` | String | No | Error description |

**Example**:

```json
{
  "feedback_id": "fb_20260108_001_a",
  "action_id": 1,
  "plan_id": "plan_20260108_001",
  "timestamp": "2026-01-08T14:32:50Z",
  "status": "executing",
  "progress_percentage": 35.5,
  "current_state": {
    "pose_x": 0.5,
    "pose_y": 0.8,
    "pose_z": 0.0,
    "heading_degrees": 45.0
  },
  "message": "Navigating to ball location... 35% complete"
}
```

**Example (Failure)**:

```json
{
  "feedback_id": "fb_20260108_001_b",
  "action_id": 1,
  "plan_id": "plan_20260108_001",
  "timestamp": "2026-01-08T14:33:05Z",
  "status": "failure",
  "progress_percentage": 50.0,
  "current_state": {
    "pose_x": 0.9,
    "pose_y": 1.5,
    "pose_z": 0.0
  },
  "message": "Navigation failed - obstacle detected",
  "error_code": "COLLISION_DETECTED",
  "error_message": "Obstacle at coordinates [1.2, 1.8]"
}
```

**Lifecycle**:
1. Created: Action feedback received
2. Streamed: Sent to user interface/TTS
3. Logged: Stored in execution history
4. Analyzed: Used for learning and optimization

---

### 6. Action Result

**Purpose**: Final outcome of action execution

**Fields**:

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `result_id` | UUID | Yes | Unique identifier for result |
| `action_id` | Integer | Yes | ID of completed action |
| `plan_id` | UUID | Yes | Reference to parent task plan |
| `success` | Boolean | Yes | Did action succeed? |
| `completion_time_ms` | Integer | Yes | Time to execute (milliseconds) |
| `final_state` | Object | No | Final robot state |
| `result_data` | Object | No | Action-specific result data |
| `error_code` | String | No | Error code if failed |
| `error_message` | String | No | Error description |
| `retry_count` | Integer | No | How many retries used |
| `timestamp` | ISO 8601 datetime | Yes | When result generated |

**Example (Success)**:

```json
{
  "result_id": "res_20260108_001_1",
  "action_id": 1,
  "plan_id": "plan_20260108_001",
  "success": true,
  "completion_time_ms": 8250,
  "final_state": {
    "pose_x": 1.5,
    "pose_y": 2.0,
    "pose_z": 0.0,
    "heading_degrees": 0.0
  },
  "result_data": {
    "distance_traveled": 2.5,
    "path_length": 2.8,
    "replans_triggered": 1
  },
  "error_code": null,
  "error_message": null,
  "retry_count": 0,
  "timestamp": "2026-01-08T14:32:58Z"
}
```

**Example (Failure)**:

```json
{
  "result_id": "res_20260108_001_2",
  "action_id": 2,
  "plan_id": "plan_20260108_001",
  "success": false,
  "completion_time_ms": 15000,
  "final_state": {
    "pose_x": 1.5,
    "pose_y": 2.0,
    "gripper_opening": 0.05
  },
  "result_data": null,
  "error_code": "GRASP_FAILED",
  "error_message": "Object slipped during grasp - insufficient grip force",
  "retry_count": 2,
  "timestamp": "2026-01-08T14:33:13Z"
}
```

---

### 7. Execution Summary

**Purpose**: Overall results of task plan execution

**Fields**:

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `execution_id` | UUID | Yes | Unique identifier for execution |
| `plan_id` | UUID | Yes | Reference to task plan |
| `success` | Boolean | Yes | Did overall task succeed? |
| `actions_completed` | Integer | Yes | Count of successful actions |
| `actions_failed` | Integer | Yes | Count of failed actions |
| `total_duration_seconds` | Float | Yes | Total execution time |
| `results` | Array[ActionResult] | Yes | Results of all actions |
| `failure_reason` | String | No | Why did task fail (if applicable) |
| `completion_percentage` | Float | Yes | % of task completed |
| `timestamp` | ISO 8601 datetime | Yes | When execution completed |

**Example**:

```json
{
  "execution_id": "exec_20260108_001",
  "plan_id": "plan_20260108_001",
  "success": false,
  "actions_completed": 2,
  "actions_failed": 1,
  "total_duration_seconds": 24.25,
  "results": [
    {"action_id": 1, "success": true, "duration_ms": 8250},
    {"action_id": 2, "success": false, "duration_ms": 15000},
    {"action_id": 3, "success": false, "duration_ms": 0, "reason": "skipped (prior action failed)"}
  ],
  "failure_reason": "Grasp action failed - object slipped",
  "completion_percentage": 66.7,
  "timestamp": "2026-01-08T14:33:15Z"
}
```

---

## Entity Relationships

### Data Flow Diagram

```
Voice Command
    ↓
    └─→ [Whisper Module] ─→ Transcribed Text
                                  ↓
                                  └─→ [GPT-4 Planning] ─→ Task Plan
                                                             ↓
                                                             └─→ [ROS 2 Executor]
                                                                  ↓
                                                      Action (1..N)
                                                                  ↓
                                                             [Robot Hardware]
                                                                  ↓
                                                    Execution Feedback (1..M per Action)
                                                                  ↓
                                                           Action Result
                                                                  ↓
                                                       Execution Summary
```

### Relationships Table

| Entity 1 | Relationship | Entity 2 | Cardinality | Notes |
|----------|-------------|----------|-------------|-------|
| Voice Command | → | Transcribed Text | 1:1 | One transcription per command |
| Transcribed Text | → | Task Plan | 1:1 | One plan per transcription |
| Task Plan | → | ROS 2 Action | 1:N | Multiple actions per plan |
| ROS 2 Action | → | Execution Feedback | 1:M | Multiple feedback events per action |
| ROS 2 Action | → | Action Result | 1:1 | One result per action |
| Task Plan | → | Execution Summary | 1:1 | One summary per plan execution |

---

## Storage & Serialization

### JSON Schema Format

All entities are serializable to JSON for:
- Inter-process communication (ROS 2 topics/services)
- Logging and audit trails
- API responses
- Database persistence

### Example: Complete Message Pipeline

```json
{
  "voice_command": {
    "command_id": "cmd_001",
    "audio_data": "...",
    "timestamp": "2026-01-08T14:32:45Z"
  },
  "transcribed_text": {
    "transcription_id": "trans_001",
    "command_id": "cmd_001",
    "text": "Pick up the blue ball",
    "confidence_score": 0.96
  },
  "task_plan": {
    "plan_id": "plan_001",
    "transcription_id": "trans_001",
    "goal": "Pick up the blue ball",
    "action_sequence": [
      {"action_id": 1, "action": "navigate", ...},
      {"action_id": 2, "action": "grasp", ...},
      {"action_id": 3, "action": "navigate", ...}
    ],
    "feasibility": "yes",
    "confidence_score": 0.92
  },
  "execution_summary": {
    "execution_id": "exec_001",
    "plan_id": "plan_001",
    "success": true,
    "actions_completed": 3,
    "actions_failed": 0,
    "total_duration_seconds": 24.25,
    "results": [...]
  }
}
```

---

## Implementation Notes

### Python Type Hints

All entities are defined with full type hints for IDE support:

```python
from dataclasses import dataclass
from typing import Optional, List
from datetime import datetime
from uuid import UUID

@dataclass
class VoiceCommand:
    command_id: str
    timestamp: datetime
    audio_data: bytes
    audio_format: str
    duration_seconds: float
    sample_rate: int
    language: Optional[str] = None
    user_id: Optional[str] = None

@dataclass
class TaskPlan:
    plan_id: str
    transcription_id: str
    goal: str
    action_sequence: List['ROS2Action']
    feasibility: str  # "yes" | "no" | "partial"
    confidence_score: float
    estimated_duration_seconds: float
    # ... more fields
```

### ROS 2 Message Definitions

All entities have corresponding ROS 2 `.msg` files:

```
vla_msgs/
├── VoiceCommand.msg
├── TranscribedText.msg
├── TaskPlan.msg
├── Action.msg
├── ExecutionFeedback.msg
└── ActionResult.msg
```

---

## Performance Considerations

### Typical Sizes

| Entity | Typical Size | Max Size |
|--------|--------------|----------|
| Voice Command | 50-500 KB | 25 MB (API limit) |
| Transcribed Text | 100-500 B | 10 KB |
| Task Plan | 1-5 KB | 50 KB |
| Action | 200-500 B | 10 KB |
| Execution Feedback | 300-800 B | 10 KB |
| Execution Summary | 1-10 KB | 100 KB |

### Latency Requirements

- Voice capture → Whisper: <2 seconds
- Transcription → GPT-4 planning: <3 seconds
- Plan generation → ROS 2 execution: <100ms
- Action execution → Result: 1-30 seconds (action-dependent)
- **Total voice-to-action**: <3 seconds (Module 4 target)

---

## Extensibility

The data model is designed to support future enhancements:

- **Multi-language support**: Add language field to all text entities
- **Multi-robot coordination**: Add robot_id to all execution entities
- **Learning & adaptation**: Add confidence feedback loop from execution results to planning
- **Cost tracking**: Add API cost and execution cost fields
- **Detailed telemetry**: Add sensor data and internal states for debugging

---

## References

- **Voice Command**: Chapter 1 - Voice-to-Action Pipeline
- **Task Plan**: Chapter 2 - LLM Cognitive Planning
- **ROS 2 Action**: Chapter 3 - ROS 2 Action Execution
- **Execution**: Chapter 4 - Capstone Project Integration

---

**Data Model Version**: 1.0
**Last Updated**: 2026-01-08
**Status**: Ready for Implementation
