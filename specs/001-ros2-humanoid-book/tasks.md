# Tasks: ROS 2 as the Robotic Nervous System for Humanoid Robots

**Input**: Design documents from `/specs/001-ros2-humanoid-book/`
**Prerequisites**: plan.md ✅, spec.md ✅, checklists/requirements.md ✅
**Feature Branch**: `001-ros2-humanoid-book`

**Organization**: Tasks organized by user story (P1, P2) to enable independent implementation and testing. Each user story is a complete, independently testable increment. Parallel execution opportunities marked with [P].

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Initialize Docusaurus book project, configure structure, and set up documentation infrastructure

**Duration**: ~2 hours | **Parallelizable**: Yes

- [x] T001 Initialize Docusaurus 3.x project in root with Node.js dependencies (package.json, docusaurus.config.js, sidebars.js)
- [x] T002 [P] Create docs/ directory structure: part1-foundations/, part2-communication/, part3-robot-structure/, examples/, urdf/
- [x] T003 [P] Create docs/setup-guide.md with Ubuntu 22.04 LTS ROS 2 Humble/Jazzy installation instructions (version-pinned)
- [x] T004 [P] Create docs/00-introduction.md as book landing page with overview of three chapters and learning path
- [x] T005 Configure sidebars.js to organize chapters into three logical parts with nested document structure
- [x] T006 [P] Configure docusaurus.config.js for GitHub Pages deployment and site metadata (title, description, repo link)
- [x] T007 [P] Create .github/workflows/deploy.yml for automated Docusaurus build and GitHub Pages deployment
- [x] T008 Create docs/examples/ subdirectories: ch1-dds-pubsub/, ch2-communication-patterns/, ch3-urdf-simulation/
- [x] T009 Create docs/urdf/ directory and add placeholder for humanoid-robot.urdf file

**Checkpoint**: ✅ Docusaurus project structure initialized, build succeeds, sidebar navigation configured

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Establish content skeleton, examples framework, and verification infrastructure

**Duration**: ~3 hours | **Parallelizable**: Yes, but T010 must complete before others

**⚠️ CRITICAL**: These tasks establish the common content patterns and example template that all chapters will follow. No chapter writing can begin until this phase is complete.

- [x] T010 [P] Create docs/examples/EXAMPLE_TEMPLATE.md documenting standard structure for all code examples: metadata, imports, setup, main code, expected output, explanation of key concepts
- [x] T011 [P] Create docs/examples/VERIFICATION_CHECKLIST.md with checklist template: "Does this example (a) run without modification on clean Ubuntu 22.04, (b) cite official ROS 2 documentation, (c) include all imports, (d) include output verification instructions?"
- [x] T012 [P] Research and document ROS 2 Humble vs. Jazzy compatibility in specs/001-ros2-humanoid-book/research.md: API stability, breaking changes, recommendation for book version
- [x] T013 [P] Research DDS Quality of Service (QoS) best practices in research.md: reliability settings for 100+ Hz sensors, history policies, deadline constraints, humanoid-specific recommendations
- [x] T014 [P] Research Gazebo + RViz integration for ROS 2 in research.md: plugin format, joint state publishing, sensor simulation, physics constraints
- [x] T015 [P] Research URDF validation tools (check_urdf, urdf_to_graphiz) and create validation script in docs/examples/validate-urdf.sh
- [x] T016 [P] Create specs/001-ros2-humanoid-book/data-model.md documenting chapter outline structure, example dependencies, and entity relationships (Nodes, Topics, Services, Actions, URDF)

**Checkpoint**: ✅ Content framework established, example templates defined, research complete, verification infrastructure ready

---

## Phase 3: User Story 1 - Understand ROS 2 Fundamentals and DDS (Priority: P1) 🎯 MVP

**Goal**: Readers understand ROS 2 as essential middleware for humanoid robots, grasp DDS architecture, publish-subscribe patterns, and can trace a complete message flow with QoS considerations

**Independent Test**: Reader can explain DDS/ROS 2 relationship, describe pub-sub pattern with humanoid example, identify 3 DDS advantages, understand QoS effects on reliability

**Duration**: ~8 hours | **Parallelizable**: Yes (Ch1a, Ch1b, Ch1c can be written in parallel, but should be reviewed before chapter building)

### Content Development for User Story 1

- [x] T017 [P] [US1] Write docs/part1-foundations/01-ros2-overview.md: "What is ROS 2?" - explanation of middleware, comparison to monolithic vs. distributed architecture, why humanoids need ROS 2 (FR-101 implementation)
- [x] T018 [P] [US1] Write docs/part1-foundations/02-dds-concepts.md: "DDS Middleware Backbone" - DDS definition, publish-subscribe model, Quality of Service settings (reliability, history, deadlines), why loose coupling benefits robotics (FR-102, FR-103, FR-206 implementation)
- [x] T019 [P] [US1] Write docs/part1-foundations/03-why-humanoids.md: "ROS 2 for Humanoid Robots" - concrete humanoid examples (IMU sensor publishing to planning/stabilization/logging subscribers), decoupled control architecture, command flow (FR-104 implementation)
- [x] T020 [US1] Create docs/examples/ch1-dds-pubsub/minimal-publisher.py: Minimal rclpy publisher that streams dummy sensor data at 100 Hz with QoS settings (FR-105, FR-207 implementation; MUST cite official rclpy documentation)
- [x] T021 [US1] Create docs/examples/ch1-dds-pubsub/minimal-subscriber.py: Minimal rclpy subscriber listening to publisher topic with verification (message count, arrival rate) (FR-105 implementation)
- [x] T022 [US1] Create docs/examples/ch1-dds-pubsub/qos-settings.py: Example showing effect of QoS settings (RELIABLE vs. BEST_EFFORT, history policies) on message delivery with explanatory comments (FR-206 implementation)
- [x] T023 [US1] Create docs/examples/ch1-dds-pubsub/node-lifecycle.py: Example demonstrating node lifecycle (initialization, spinning, shutdown phases) with clear comments (FR-105 implementation)
- [x] T024 [US1] Create docs/examples/ch1-dds-pubsub/humanoid-imu-example.py: Realistic humanoid example: IMU sensor publisher → multiple subscribers (planning, stabilization) (FR-104, FR-105 implementation)
- [x] T025 [US1] Add setup instructions in docs/part1-foundations/ linking to docs/setup-guide.md; include section "How to Run Chapter 1 Examples" with step-by-step instructions (FR-404, FR-405 implementation)
- [x] T026 [US1] Create docs/examples/ch1-dds-pubsub/README.md documenting all Chapter 1 examples: what each demonstrates, expected output, how to run, known issues (FR-405, FR-402 implementation)
- [x] T027 [US1] [P] Verify all Chapter 1 code examples against checklist: run on clean Ubuntu 22.04, cite official documentation, include all imports/context, output verification works (FR-401, FR-403 implementation)

**Internal Dependencies**: T020-T026 can run in parallel; T027 must wait for all examples complete

**Checkpoint**: ✅ Chapter 1 (Foundations) complete - readers understand ROS 2, DDS, pub-sub patterns, QoS, and node lifecycle. All 5 working examples verified to run independently on clean Ubuntu 22.04. Story 1 is self-contained and testable.

---

## Phase 4: User Story 2 - Master ROS 2 Communication Patterns (Priority: P1)

**Goal**: Readers master Topics (async pub-sub), Services (request-reply), and Actions (async tasks) through hands-on rclpy examples. Understand agent/controller architecture for humanoid systems. Can implement multi-node communication patterns.

**Independent Test**: Developer creates working rclpy nodes, publishes joint state at 100 Hz, implements service server, runs multi-node agent/controller example successfully

**Duration**: ~12 hours | **Parallelizable**: Yes (documentation chapters, then examples in parallel)

### Content Development for User Story 2

- [x] T028 [P] [US1] Write docs/part2-communication/04-nodes-and-lifecycle.md: "Understanding Nodes" - node definition, lifecycle (initialization, spinning, shutdown), executor patterns, how nodes form distributed system (FR-201 implementation)
- [x] T029 [P] [US1] Write docs/part2-communication/05-topics-pubsub.md: "Topics: Async Pub-Sub" - topic concept, message types (sensor_msgs/JointState, etc.), subscriber/publisher patterns, humanoid examples (joint states, motor commands), realistic frequency requirements (100+ Hz sensors) (FR-202, FR-207 implementation)
- [x] T030 [P] [US1] Write docs/part2-communication/06-services-reqrep.md: "Services: Synchronous Request-Reply" - service definition, client/server patterns, timeouts, humanoid examples (set motor speed, grasp commands) (FR-203 implementation)
- [x] T031 [P] [US1] Write docs/part2-communication/07-actions-async.md: "Actions: Long-Running Tasks" - action concept, goals, feedback, results, humanoid examples (move to pose, reach target) (FR-204 implementation)
- [x] T032 [P] [US1] Write docs/part2-communication/08-agent-controller-example.md: "Agent/Controller Pattern for Humanoid Control" - comprehensive example showing planning node → sensor publisher ↔ motor controller with Topics, Services, and Actions integrated (FR-205 implementation)
- [ ] T033 [US1] Create docs/examples/ch2-communication-patterns/topic-publisher.py: rclpy topic publisher with realistic sensor data (joint states) at 100 Hz, with QoS settings (FR-202, FR-207 implementation)
- [ ] T034 [US1] Create docs/examples/ch2-communication-patterns/topic-subscriber.py: rclpy topic subscriber with message arrival verification and frequency measurement (FR-202, FR-207 implementation)
- [ ] T035 [US1] Create docs/examples/ch2-communication-patterns/pubsub-multi-node.py: Complete working example: 2 nodes communicating via topic (publisher/subscriber pattern) with verification (FR-205 implementation)
- [ ] T036 [US1] Create docs/examples/ch2-communication-patterns/service-server.py: Service server implementing motor speed command (receives motor_id and speed, returns confirmation) (FR-203 implementation)
- [ ] T037 [US1] Create docs/examples/ch2-communication-patterns/service-client.py: Service client calling motor speed service with timeout handling (FR-203 implementation)
- [ ] T038 [US1] Create docs/examples/ch2-communication-patterns/service-multi-node.py: Complete working example: service client/server communication (FR-205 implementation)
- [ ] T039 [US1] Create docs/examples/ch2-communication-patterns/action-server.py: Action server implementing "move to pose" task with goal, feedback, result (FR-204 implementation)
- [ ] T040 [US1] Create docs/examples/ch2-communication-patterns/action-client.py: Action client sending pose goals and processing feedback (FR-204 implementation)
- [ ] T041 [US1] Create docs/examples/ch2-communication-patterns/agent-controller-pattern.py: End-to-end humanoid control example: planning node publishes pose goals (action) → controller subscribes to goals and publishes motor commands (topic) → sensor publisher provides feedback (topic). All 3 nodes run together successfully (FR-205, FR-008 from spec implementation)
- [ ] T042 [US1] Create docs/examples/ch2-communication-patterns/humanoid-joint-state-publisher.py: Realistic humanoid example: publish joint states for all humanoid joints at 100 Hz (FR-207 implementation)
- [ ] T043 [US1] Create docs/examples/ch2-communication-patterns/humanoid-motor-control-service.py: Realistic humanoid example: service to set motor speed with joint limits validation (FR-203 implementation)
- [ ] T044 [US1] Create docs/examples/ch2-communication-patterns/README.md documenting all Chapter 2 examples, expected outputs, how to run, integration paths (FR-402, FR-405 implementation)
- [ ] T045 [US1] [P] Verify all Chapter 2 code examples: run on clean Ubuntu 22.04, publish/subscribe frequencies correct (100+ Hz sensors measured), multi-node examples work end-to-end, all cite official rclpy documentation (FR-401, FR-403, FR-207 implementation)

**Internal Dependencies**: T028-T032 can run in parallel; T033-T044 can run in parallel; T045 must wait for all examples complete

**Checkpoint**: ✅ Chapter 2 (Communication Patterns) complete - readers master Nodes, Topics, Services, Actions through ~12 working examples. Agent/controller pattern end-to-end example demonstrates complete humanoid control flow. All examples verified. Story 2 is self-contained and testable.

---

## Phase 5: User Story 3 - Describe and Simulate Humanoid Robot Structure (Priority: P2)

**Goal**: Readers understand URDF as robot structure description format, can write complete humanoid URDF with realistic joints/masses, load in RViz and Gazebo, and control via ROS 2 joint state publishing

**Independent Test**: Developer writes humanoid URDF, loads in RViz/Gazebo without errors, joint hierarchy correct, constraints applied, can control joints via ROS 2 published commands and visualize movement

**Duration**: ~8 hours | **Parallelizable**: Yes (documentation and URDF development in parallel)

### Content Development for User Story 3

- [x] T046 [P] [US2] Write docs/part3-robot-structure/09-urdf-fundamentals.md: "Understanding URDF" - URDF XML format explanation, links (bodies), joints (connections), inertia, collision geometry, sensors, actuators, how URDF connects to ROS 2 control (FR-301 implementation)
- [x] T047 [P] [US2] Write docs/part3-robot-structure/10-humanoid-urdf-example.md: "Complete Humanoid URDF" - detailed walkthrough of humanoid URDF structure: torso, arms (2), legs (2), head with realistic joint limits, link masses, sensor placements (IMU, cameras). Explains design choices and humanoid-specific constraints (FR-302 implementation)
- [x] T048 [P] [US2] Write docs/part3-robot-structure/11-rviz-gazebo-integration.md: "Visualization and Simulation" - how to load URDF in RViz (robot state publisher, joint state visualizer), how to load in Gazebo (physics plugins, sensor simulation), step-by-step walkthrough, real hardware vs. simulation differences (FR-303, FR-304, FR-306 implementation)
- [ ] T049 [US2] Create docs/urdf/humanoid-robot.urdf: Complete, working humanoid URDF file with: torso, 2 arms (3 joints each), 2 legs (3 joints each), head (1 joint), realistic link masses, joint limits, inertia tensors, collision geometries (FR-302 implementation)
- [ ] T050 [US2] Create docs/examples/ch3-urdf-simulation/simple-robot.urdf: Simplified URDF (1 link, 1 joint) as learning example to understand URDF structure before humanoid complexity (FR-301 implementation)
- [ ] T051 [US2] Create docs/examples/ch3-urdf-simulation/load-urdf-rrviz.py: rclpy example: load humanoid URDF, run robot_state_publisher, display in RViz with joint state visualization (FR-303 implementation)
- [ ] T052 [US2] Create docs/examples/ch3-urdf-simulation/joint-state-publisher.py: rclpy example: publish joint states for all humanoid joints, enabling manual joint control in RViz GUI (FR-303 implementation)
- [ ] T053 [US2] Create docs/examples/ch3-urdf-simulation/gazebo-launch.yaml: ROS 2 launch file to load humanoid URDF in Gazebo simulator with physics and joint control (FR-304 implementation)
- [ ] T054 [US2] Create docs/examples/ch3-urdf-simulation/humanoid-joint-control.py: rclpy example: subscribe to joint commands and control humanoid joints in Gazebo, demonstrating real-time control with URDF constraints (FR-305 implementation)
- [ ] T055 [US2] Create docs/examples/ch3-urdf-simulation/real-vs-simulation.md: Documentation guide on adapting examples for real hardware vs. simulation: Gazebo plugins that won't work on real robots, timing differences, safety considerations (FR-306 implementation)
- [ ] T056 [US2] Create docs/examples/ch3-urdf-simulation/validate-urdf.sh: Bash script to validate humanoid URDF using check_urdf tool, ensuring parseable format and no errors (FR-305 implementation)
- [ ] T057 [US2] Create docs/examples/ch3-urdf-simulation/README.md documenting all Chapter 3 examples, URDF structure, how to load in RViz/Gazebo, known issues, hardware adaptation notes (FR-304, FR-306, FR-405 implementation)
- [ ] T058 [US2] [P] Verify all Chapter 3 examples: humanoid URDF validates without errors, loads in RViz and Gazebo, joints move smoothly when controlled, real hardware vs. simulation guidance is clear (FR-301, FR-304, FR-305, FR-306 implementation)

**Internal Dependencies**: T046-T048 can run in parallel; T049-T057 can run in parallel; T058 must wait for all examples complete

**Checkpoint**: ✅ Chapter 3 (Robot Structure) complete - readers understand URDF format, have working humanoid URDF, can load in RViz/Gazebo, and control via ROS 2. All ~8 examples verified. Story 3 is self-contained and testable. Readers can adapt URDF for real hardware.

---

## Phase 6: Cross-Chapter Integration & Verification (Polish)

**Purpose**: Verify end-to-end book consistency, cross-chapter integration, and all success criteria

**Duration**: ~6 hours | **Blocking**: Yes - must complete before publishing

- [ ] T059 [P] Verify SC-002 - Every ROS 2 claim in all three chapters cites official documentation: grep for claims without citations, add links to docs/links.md or inline citations
- [ ] T060 [P] Verify SC-006 - 90%+ code examples include clarifying comments: audit all examples in docs/examples/, add comments explaining non-obvious behavior (QoS choices, why RELIABLE vs. BEST_EFFORT, node lifecycle steps)
- [ ] T061 [P] Verify SC-007 - Cross-chapter consistency: check terminology uniform across Ch1-Ch3, cross-chapter references accurate, examples build logically from foundations → communication → structure
- [ ] T062 [P] Create docs/examples/END-TO-END-EXAMPLE.md: Complete end-to-end humanoid control example combining all three chapters: (1) DDS pub-sub foundation, (2) agent/controller communication, (3) URDF simulation. Reader can follow this one example from setup through control verification. (FR-008 from spec implementation, SC-008 implementation)
- [ ] T063 Verify SC-001 - All code examples run without modification on clean Ubuntu 22.04: test entire docs/examples/ directory on clean VM, document any issues, fix examples, re-test
- [ ] T064 Verify SC-004 - URDF validates and runs: test docs/urdf/humanoid-robot.urdf with check_urdf, load in RViz, control in Gazebo, verify smooth movement
- [ ] T065 Verify SC-005 - Setup completes in ≤30 minutes: time docs/setup-guide.md procedures on clean Ubuntu 22.04, optimize instructions if exceeding 30 minutes
- [x] T066 [P] Add docs/references.md: Centralized list of all official ROS 2 documentation links cited in book (ROS 2 docs, rclpy API, DDS specs, URDF format, Gazebo, RViz) with descriptions
- [x] T067 [P] Create docs/known-issues.md: Document known issues, workarounds, debugging tips for all chapters (e.g., Gazebo plugin loading, RViz visualization, network ROS 2 setup) (FR-405 implementation)
- [x] T068 [P] Create docs/glossary.md: Glossary of all ROS 2 and humanoid robotics terms used in book (Node, Topic, Service, Action, URDF, DDS, QoS, etc.) with definitions (SC-009 implementation)
- [x] T069 Create docs/quickstart.md: "Get started in 30 minutes" guide: Ubuntu 22.04 setup, first working example (minimal publisher/subscriber from Chapter 1), how to run all examples, links to full chapters
- [ ] T070 [P] Build Docusaurus site: `npm run build` in project root, verify no warnings/errors, check sidebar navigation complete, all links valid
- [ ] T071 Test GitHub Pages deployment: push to branch, verify GitHub Actions workflow triggers, Docusaurus site deploys to GitHub Pages successfully, site accessible and searchable

**Internal Dependencies**: T059-T062, T066-T070 can run in parallel; T063-T065 require clean test systems; T071 final gate

**Checkpoint**: ✅ All chapters complete, verified, integrated, documented. End-to-end example working. All success criteria SC-001 to SC-010 satisfied. Book ready for publication.

---

## Summary & Delivery Plan

### Task Counts by Phase

| Phase | Count | Purpose |
|-------|-------|---------|
| Phase 1: Setup | 9 | Docusaurus initialization, structure, deployment |
| Phase 2: Foundational | 7 | Content framework, examples template, research |
| Phase 3: User Story 1 | 11 | Chapter 1 (Foundations): ROS 2 & DDS understanding |
| Phase 4: User Story 2 | 18 | Chapter 2 (Communication): Patterns & agent/controller |
| Phase 5: User Story 3 | 13 | Chapter 3 (Structure): URDF & simulation |
| Phase 6: Polish | 13 | Integration, verification, deployment |
| **Total** | **71** | Complete ROS 2 humanoid robotics book |

### Parallel Execution Opportunities

**Phase 1** (9 tasks): All can run in parallel except T005 (depends on T002)
- Parallel: T001-T004, T006-T009 (estimate: 2 hours vs. 1 hour sequential)

**Phase 2** (7 tasks): T010 must complete first, then T011-T016 parallel
- T010 → T011-T016 parallel (estimate: 3 hours vs. 1.5 hours sequential)

**Phase 3** (11 tasks): T017-T019 (documentation) and T020-T026 (examples) can run in parallel; T027 verification final
- Parallel: T017-T026 → T027 (estimate: 8 hours vs. 4 hours sequential)

**Phase 4** (18 tasks): T028-T032 (documentation) and T033-T044 (examples) parallel; T045 verification final
- Parallel: T028-T044 → T045 (estimate: 12 hours vs. 6 hours sequential)

**Phase 5** (13 tasks): T046-T048 (documentation) and T049-T057 (URDF/examples) parallel; T058 verification final
- Parallel: T046-T057 → T058 (estimate: 8 hours vs. 4 hours sequential)

**Phase 6** (13 tasks): Most can run in parallel; T063-T065 require clean test systems; T071 final gate
- Parallel: T059-T062, T066-T070 → T063-T065 (sequential) → T071 (estimate: 6 hours, sequential due to testing)

### MVP Scope (Minimum Viable Product)

To deliver value with minimal effort:

**Recommended MVP: Complete Phase 1 + Phase 2 + Phase 3 (User Story 1 only)**

This delivers:
- ✅ Docusaurus book site initialized and deployed
- ✅ Chapter 1: ROS 2 Fundamentals & DDS (5 working examples)
- ✅ Readers understand pub-sub, DDS, QoS, node lifecycle
- ✅ All Chapter 1 examples reproducible on clean Ubuntu 22.04
- ✅ Setup takes ≤30 minutes

**Effort**: ~19 tasks (Phases 1-3) = ~12 hours total effort

**Next Increments**:
- +Phase 4: Add Chapter 2 (Communication Patterns) = +18 tasks
- +Phase 5: Add Chapter 3 (URDF & Simulation) = +13 tasks
- +Phase 6: Polish & full verification = +13 tasks

### Independent Test Criteria (by User Story)

**User Story 1 Complete**: Reader can explain DDS/ROS 2 relationship, describe pub-sub pattern with humanoid example, run all Chapter 1 examples successfully on clean Ubuntu 22.04

**User Story 2 Complete**: Developer creates working rclpy nodes, publishes sensor data at 100 Hz, implements services, runs multi-node agent/controller example successfully, all on clean Ubuntu 22.04

**User Story 3 Complete**: Developer writes/uses humanoid URDF, loads in RViz and Gazebo without errors, controls joints via ROS 2 published commands, visualizes smooth movement

### Dependency Graph

```
Phase 1 (Setup)
    ↓
Phase 2 (Foundational)
    ↓
Phase 3 (US1: Foundations) ────┐
    ↓                           ├──→ Phase 6 (Polish & Verification)
Phase 4 (US2: Communication) ──┤
    ↓                           ├──→ GitHub Pages Deployment
Phase 5 (US3: Structure) ───────┘
```

User Stories 3, 4, 5 (Phases 3, 4, 5) are sequentially dependent in priority order but can run in parallel per story.

---

## Implementation Strategy Notes

1. **Specification Adherence**: Every task references applicable functional requirements (FR-xxx) and success criteria (SC-xxx) from spec.md
2. **Constitution Compliance**: All code examples cite official ROS 2 documentation (SC-002, PR Principle II: Technical Accuracy)
3. **Reproducibility**: All examples tested on clean Ubuntu 22.04 (SC-001, SC-005, PR Principle IV: Reproducible Setup)
4. **Code Quality**: Examples include complete imports, setup, and expected output (SC-006, SC-009, FR-402)
5. **Testing**: No automated tests in this phase (documentation project); manual verification via examples running successfully

---

## Success Criteria Mapping

| SC | Task Coverage | Verification |
|----|---------------|----|
| SC-001 | T063 (all examples on clean Ubuntu 22.04) | All docs/examples/ pass |
| SC-002 | T059 (citation verification) + all content tasks | grep for claims without sources |
| SC-003 | T045, T058 (user story verification) | 80%+ reader success in independent tests |
| SC-004 | T064 (URDF validation) | check_urdf passes, RViz/Gazebo load, joints move |
| SC-005 | T065 (setup time) | Time docs/setup-guide.md on clean system |
| SC-006 | T060 (comment audit) | 90%+ code examples have clarifying comments |
| SC-007 | T061 (consistency check) | Terminology, references, building blocks verified |
| SC-008 | T062 (end-to-end example) | Complete example from setup to control |
| SC-009 | T068 (glossary) + labeling in content | Clear distinction: concepts vs. examples vs. simulation |
| SC-010 | T066 (references) + all examples | Version-locked dependencies documented |

**Ready for Implementation**: All tasks are specific, actionable, and can begin immediately. Each task has clear file paths and acceptance criteria.
