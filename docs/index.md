---
sidebar_position: 1
---

<div class="home-container">

{/* ============================================================================
   HERO SECTION
   ============================================================================ */}

<section class="hero-section">
  <div class="hero-content">
    <h1 class="hero-title">ROS 2 for Humanoid Robots</h1>
    <p class="hero-subtitle">Learn distributed control systems with hands-on examples and complete humanoid robotics projects</p>
    <div class="hero-cta">
      <a href="./setup-guide" class="cta-button cta-primary">Get Started</a>
      <a href="#modules" class="cta-button cta-secondary">Explore Modules</a>
    </div>
  </div>
</section>

{/* ============================================================================
   MODULE CARDS SECTION
   ============================================================================ */}

<section class="modules-section" id="modules">
  <h2 class="section-title">Four Modules. One Goal.</h2>
  <p class="section-subtitle">Master ROS 2 from fundamentals to complete humanoid simulations</p>

  <div class="modules-grid">
    {/* Module 1 */}
    <div class="module-card">
      <div class="module-header module-1">
        <div class="module-number">1</div>
      </div>
      <div class="module-body">
        <h3 class="module-title">Humanoid Control</h3>
        <p class="module-description">Understand ROS 2 as middleware and DDS concepts essential for distributed humanoid systems</p>
        <ul class="module-topics">
          <li>ROS 2 overview and architecture</li>
          <li>DDS concepts and QoS settings</li>
          <li>Why humanoids need distributed systems</li>
        </ul>
        <div class="module-meta">
          <span class="module-duration">~2 hours</span>
          <a href="./part1-foundations/ros2-overview" class="module-link">Start Module →</a>
        </div>
      </div>
    </div>

    {/* Module 2 */}
    <div class="module-card">
      <div class="module-header module-2">
        <div class="module-number">2</div>
      </div>
      <div class="module-body">
        <h3 class="module-title">Perception & SLAM</h3>
        <p class="module-description">Master communication patterns with Topics, Services, and Actions for real humanoid control</p>
        <ul class="module-topics">
          <li>Nodes and lifecycle management</li>
          <li>Topics, Services, and Actions</li>
          <li>Agent/Controller architecture</li>
        </ul>
        <div class="module-meta">
          <span class="module-duration">~4 hours</span>
          <a href="./part2-communication/nodes-and-lifecycle" class="module-link">Start Module →</a>
        </div>
      </div>
    </div>

    {/* Module 3 */}
    <div class="module-card">
      <div class="module-header module-3">
        <div class="module-number">3</div>
      </div>
      <div class="module-body">
        <h3 class="module-title">Isaac Brain</h3>
        <p class="module-description">Learn URDF to describe complete humanoid robots and visualize them in RViz</p>
        <ul class="module-topics">
          <li>URDF format and structure</li>
          <li>Humanoid robot descriptions</li>
          <li>RViz visualization</li>
        </ul>
        <div class="module-meta">
          <span class="module-duration">~3 hours</span>
          <a href="./part3-robot-structure/urdf-fundamentals" class="module-link">Start Module →</a>
        </div>
      </div>
    </div>

    {/* Module 4 */}
    <div class="module-card">
      <div class="module-header module-4">
        <div class="module-number">4</div>
      </div>
      <div class="module-body">
        <h3 class="module-title">Vision-Language-Action</h3>
        <p class="module-description">Simulate humanoid robots with physics and sensors using Gazebo and ROS 2</p>
        <ul class="module-topics">
          <li>Gazebo physics simulation</li>
          <li>Sensor integration</li>
          <li>Complete control simulation</li>
        </ul>
        <div class="module-meta">
          <span class="module-duration">~5 hours</span>
          <a href="./part4-gazebo-simulation/gazebo-fundamentals" class="module-link">Start Module →</a>
        </div>
      </div>
    </div>
  </div>
</section>

{/* ============================================================================
   QUICK START SECTION
   ============================================================================ */}

<section class="quickstart-section">
  <h2 class="section-title">Get Started in 30 Minutes</h2>

  <div class="quickstart-grid">
    <div class="quickstart-card">
      <div class="step-number">1</div>
      <h3>Setup Environment</h3>
      <p>Install ROS 2 Humble and dependencies following our setup guide</p>
      <a href="./setup-guide" class="card-link">View Setup Guide</a>
    </div>

    <div class="quickstart-card">
      <div class="step-number">2</div>
      <h3>Run First Example</h3>
      <p>Write and run your first ROS 2 publisher node in 10 minutes</p>
      <a href="./quickstart" class="card-link">Try Quickstart</a>
    </div>

    <div class="quickstart-card">
      <div class="step-number">3</div>
      <h3>Explore Foundations</h3>
      <p>Learn ROS 2 concepts with hands-on examples and humanoid scenarios</p>
      <a href="./part1-foundations/ros2-overview" class="card-link">Start Learning</a>
    </div>
  </div>
</section>

{/* ============================================================================
   LEARNING PATH SECTION
   ============================================================================ */}

<section class="learning-path-section">
  <h2 class="section-title">Your Learning Journey</h2>

  <div class="learning-path">
    <div class="path-step">
      <div class="path-number">Foundation</div>
      <p>Understand ROS 2 concepts, middleware, and why humanoids use distributed systems</p>
    </div>

    <div class="path-arrow">→</div>

    <div class="path-step">
      <div class="path-number">Practice</div>
      <p>Build multi-node systems with Topics, Services, and Actions patterns</p>
    </div>

    <div class="path-arrow">→</div>

    <div class="path-step">
      <div class="path-number">Integration</div>
      <p>Describe humanoid robots with URDF and simulate complete control systems</p>
    </div>
  </div>
</section>

{/* ============================================================================
   KEY FEATURES SECTION
   ============================================================================ */}

<section class="features-section">
  <h2 class="section-title">Why Learn From This Book</h2>

  <div class="features-grid">
    <div class="feature">
      <div class="feature-icon">✓</div>
      <h3>100% Reproducible</h3>
      <p>Every code example runs on clean Ubuntu 22.04 without modification</p>
    </div>

    <div class="feature">
      <div class="feature-icon">✓</div>
      <h3>Officially Sourced</h3>
      <p>All claims cite ROS 2 docs and DDS specifications</p>
    </div>

    <div class="feature">
      <div class="feature-icon">✓</div>
      <h3>Hands-On Learning</h3>
      <p>30+ working Python examples you can run immediately</p>
    </div>

    <div class="feature">
      <div class="feature-icon">✓</div>
      <h3>Humanoid-Focused</h3>
      <p>Real scenarios: joint states, motor control, humanoid URDF</p>
    </div>

    <div class="feature">
      <div class="feature-icon">✓</div>
      <h3>Clear Progression</h3>
      <p>Concepts → Patterns → Complete Systems (Part 1 → 2 → 3 → 4)</p>
    </div>

    <div class="feature">
      <div class="feature-icon">✓</div>
      <h3>Hardware Ready</h3>
      <p>Learn what works in simulation and what requires real hardware</p>
    </div>
  </div>
</section>

{/* ============================================================================
   TARGET AUDIENCE SECTION
   ============================================================================ */}

<section class="audience-section">
  <h2 class="section-title">Who Should Learn This?</h2>

  <div class="audience-grid">
    <div class="audience-item">
      <h3>AI & ML Engineers</h3>
      <p>Building vision and language models for humanoid robots</p>
    </div>

    <div class="audience-item">
      <h3>Robotics Developers</h3>
      <p>With intermediate Python skills and distributed systems knowledge</p>
    </div>

    <div class="audience-item">
      <h3>Students</h3>
      <p>Learning robotics from scratch with hands-on projects</p>
    </div>

    <div class="audience-item">
      <h3>System Architects</h3>
      <p>Designing control architectures for complex humanoid systems</p>
    </div>
  </div>
</section>

{/* ============================================================================
   TECH STACK SECTION
   ============================================================================ */}

<section class="tech-section">
  <h2 class="section-title">Tech Stack</h2>

  <div class="tech-stack">
    | **Component** | **Tool** | **Version** |
    |---|---|---|
    | **ROS 2** | Humble (LTS) | 2022.12+ |
    | **Python** | rclpy | 3.10+ |
    | **Robotics** | URDF, RViz2, Gazebo | Standard |
    | **OS** | Ubuntu | 22.04 LTS |
  </div>
</section>

{/* ============================================================================
   CTA SECTION
   ============================================================================ */}

<section class="final-cta-section">
  <h2>Ready to Build Humanoid Robots?</h2>
  <p>Start learning ROS 2 with real hands-on examples</p>

  <div class="final-cta">
    <a href="./setup-guide" class="cta-button cta-primary">Start Learning</a>
    <a href="https://github.com/anthropics/ros2-humanoid-book" class="cta-button cta-secondary">View on GitHub</a>
  </div>
</section>

</div>
