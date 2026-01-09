# Code Example Verification Checklist

Use this checklist to verify every code example in the book before it's considered complete.

## Reproducibility Checklist

- [ ] **Clean System Test**: Example runs on fresh Ubuntu 22.04 LTS + ROS 2 Humble without any modifications
- [ ] **All Imports Included**: No missing `import` statements; reader can copy-paste and run immediately
- [ ] **No External Services**: Example runs without needing external servers, cloud services, or third-party APIs
- [ ] **Dependencies Documented**: All required packages listed (e.g., `sensor_msgs`, `rclpy`)
- [ ] **Setup Instructions Clear**: Step-by-step; beginner-friendly; no steps assumed
- [ ] **Expected Output Accurate**: Output section matches actual output when run (check timestamp formats, exact messages)
- [ ] **Copy-Paste Ready**: Code can be copied directly from Markdown into file and run without modification

## Code Quality Checklist

- [ ] **Syntax Correct**: Code passes Python linter (no syntax errors, proper indentation)
- [ ] **Imports Organized**: Standard library first, then third-party (rclpy), then relative imports
- [ ] **Comments Explain 'Why'**: Comments explain purpose, not obvious code (`#bad: i = i + 1`, `#good: increment counter for next cycle`)
- [ ] **Variable Names Descriptive**: `publisher` not `p`, `joint_state_msg` not `msg`
- [ ] **Functions/Classes Named Well**: `publish_message()` not `do_thing()`
- [ ] **No Magic Numbers**: Explain hardcoded values (e.g., `timer_period = 0.01 # 100 Hz`)
- [ ] **Error Handling**: Graceful handling of Ctrl+C (cleanup with `destroy_node()`, `shutdown()`)
- [ ] **Logging Used**: `self.get_logger().info()` for important events, not print statements
- [ ] **No Hardcoded Paths**: Relative paths or assumed running from example directory

## Documentation Checklist

- [ ] **Title Clear**: Describes what the example does (not "Example 1", but "Minimal Publisher")
- [ ] **Description Complete**: 2-3 sentences explaining purpose and learning value
- [ ] **Concepts Listed**: What ROS 2 concepts does this teach?
- [ ] **Difficulty Level Stated**: Beginner, Intermediate, or Advanced?
- [ ] **Time Estimate Included**: Realistic time to understand and run example (5-15 minutes typical)
- [ ] **Files Listed**: All files needed (e.g., `.py` script, `.yaml` config, `.urdf`)
- [ ] **Prerequisites Stated**: What must be installed/understood before this example?
- [ ] **How to Run Instructions**: Step-by-step, starting from clone/navigate to directory
- [ ] **Expected Output Shown**: Exact output readers should see when run correctly
- [ ] **Common Errors Documented**: Table of "error message" → "cause" → "fix"
- [ ] **Explanation Detailed**: Break down code into logical sections with rationale
- [ ] **Key Concepts Reinforced**: Bullet list of concepts readers learn from this
- [ ] **Citations Included**: Links to official ROS 2 documentation for every claim
- [ ] **Next Steps Suggested**: Where to go after mastering this example

## Official Source Compliance Checklist

- [ ] **All Claims Sourced**: Every technical statement traces back to official documentation
- [ ] **ROS 2 API Accurate**: Function signatures, parameter names, return types match official rclpy docs
- [ ] **Message Types Standard**: Using official message types (sensor_msgs, geometry_msgs, etc.)
- [ ] **QoS Settings Explained**: If using QoS, explain why (reliability, history, deadlines)
- [ ] **No Deprecated APIs**: Not using removed or deprecated ROS 2 functions
- [ ] **Documentation Links Work**: Citations are active, not broken links
- [ ] **Versions Specified**: Indicate ROS 2 version (Humble, Jazzy) and rclpy version tested

## Humanoid-Specific Checklist

- [ ] **Humanoid Examples Realistic**: Not generic; uses humanoid robot scenarios (joint names, joint counts, sensor placement)
- [ ] **Joint Names Appropriate**: Uses real humanoid joint conventions (e.g., `right_shoulder_pitch`, not `motor_1`)
- [ ] **Frequencies Realistic**: Sensor publishing at realistic rates (100+ Hz for IMU, etc.)
- [ ] **Constraints Documented**: Joint limits, actuator constraints, real hardware limitations mentioned
- [ ] **Real vs. Simulation Clear**: Explicitly states if example is simulation-only or works on real hardware

## Testing Checklist

- [ ] **Tested on Fresh VM**: Run on completely new Ubuntu 22.04 LTS installation
- [ ] **After ROS 2 Install**: Tested after running setup-guide.md, not with pre-existing ROS 2 installs
- [ ] **Multiple Runs**: Verified works on at least 2 separate test runs
- [ ] **Ctrl+C Shutdown Clean**: Gracefully stops when user presses Ctrl+C, no error traces
- [ ] **Handles No Subscribers**: Works even if no other nodes are listening (no error messages about missing subscribers)
- [ ] **Timeout Handling**: Services/actions have reasonable timeouts, don't hang forever if no response
- [ ] **Resource Cleanup**: Doesn't consume excessive memory/CPU over time (no memory leaks)

## Integration Checklist

- [ ] **Part of Learning Progression**: Fits within chapter's learning goals (Part 1 → 2 → 3)
- [ ] **Builds on Prior Examples**: References or extends previous examples where relevant
- [ ] **Independent Test**: Can be run and understood independently (doesn't require prior examples)
- [ ] **Cross-Chapter Consistency**: Terminology matches Chapter 1, 2, 3 consistent style
- [ ] **File Naming Consistent**: Follows pattern of other examples in same chapter

## Final Sign-Off Checklist

- [ ] **All Above Passed**: Every checkbox above is checked
- [ ] **Code Reviewed**: At least one peer review confirms quality
- [ ] **Tested on Target System**: Works on Ubuntu 22.04 + ROS 2 Humble (exact versions used in book)
- [ ] **Documentation Complete**: README exists with full context
- [ ] **Ready for Publication**: No TODO comments, no "fix this later" notes in code or docs

---

## Verification Process

1. **Run the example** on fresh Ubuntu 22.04 + ROS 2 Humble (use VM if needed)
2. **Check output** matches "Expected Output" section exactly
3. **Try variations** listed in "Try This" section; they should work as described
4. **Read all documentation** line-by-line; it should be clear to a beginner
5. **Verify citations** by following each link to official ROS 2 documentation
6. **Check for common errors**: Do the listed errors match what would occur? Are fixes correct?
7. **Sign off**: If all items checked, example is ready for publication

---

## Example Verification Template

For each example, fill in:

```markdown
### [Example Name]

- [ ] Reproducibility: PASS / FAIL
- [ ] Code Quality: PASS / FAIL
- [ ] Documentation: PASS / FAIL
- [ ] Official Sources: PASS / FAIL
- [ ] Humanoid-Specific: PASS / FAIL
- [ ] Testing: PASS / FAIL
- [ ] Integration: PASS / FAIL
- [ ] Final Sign-Off: PASS / FAIL

Tested on: Ubuntu 22.04 LTS, ROS 2 Humble
Date Verified: YYYY-MM-DD
Verified By: [Name]

Issues Found: [None or list specific issues]
```

---

## Quick Verification (5 minutes)

If you're short on time, do this minimum verification:

1. Run the example: `python3 example-name.py`
2. Check output matches expected output
3. Verify imports are all present
4. Scan comments; at least 2-3 explaining non-obvious behavior
5. Confirm one citation link works
6. Confirm Ctrl+C shuts down cleanly

If all 6 steps pass, example is likely good. Full verification should still be done before publication.

---

This checklist ensures all examples meet the book's standards for reproducibility, clarity, and accuracy.
