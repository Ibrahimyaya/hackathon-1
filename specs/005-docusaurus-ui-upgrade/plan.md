# Implementation Plan: 005-Docusaurus UI Upgrade with Modern Design

**Branch**: `005-docusaurus-ui-upgrade` | **Date**: 2026-01-08 | **Spec**: [specs/005-docusaurus-ui-upgrade/spec.md](spec.md)
**Input**: Feature specification from `/specs/005-docusaurus-ui-upgrade/spec.md`

**Command**: `/sp.plan` workflow

## Summary

Modernize the Docusaurus-based educational robotics documentation with a clean, accessible UI featuring module-wise navigation, responsive design (320px-1440px+), WCAG AA compliance, dark/light theming via CSS variables, and configurable design tokens. Preserve all content; enhance discoverability and user experience through information architecture, typography system, color system, spacing system, and responsive breakpoints.

## Technical Context

**Language/Version**: JavaScript/TypeScript (Docusaurus 2.x, Node.js 18+)
**Primary Dependencies**: Docusaurus 2.x (React 17+), CSS-in-JS (CSS Modules or Tailwind CSS), markdown/MDX
**Storage**: N/A (static site generation)
**Testing**: Jest (unit), Cypress/Playwright (e2e), Lighthouse (a11y/performance)
**Target Platform**: Web (Chrome, Firefox, Safari, Edge; all latest 2 versions)
**Project Type**: Static website (Docusaurus site with custom theming)
**Performance Goals**: ≤20% slower than baseline; First Contentful Paint <2s; Lighthouse score ≥85
**Constraints**: Docusaurus theming compatibility; no custom build steps; CSS variable support in browsers; WCAG AA compliance (4.5:1 contrast)
**Scale/Scope**: 4 modules, 20-30 chapters; 320px to 1440px+ responsive breakpoints; 5 user stories; 24 functional requirements

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Spec-First Workflow**: ✅ Specification exists (20/20 quality checklist passed); ADR will document theming architecture decision.
**Technical Accuracy**: ✅ All requirements based on Docusaurus 2.x official docs, WCAG AA official standards, CSS variables (W3C standard).
**Developer-Focused Writing**: ✅ Plan includes quickstart.md for theme customization with concrete examples; no marketing language.
**Reproducible Setup**: ✅ Theming changes use standard Docusaurus CSS injection; no custom build steps required.
**Stack Fidelity**: ✅ Uses Docusaurus 2.x per constitution; no alternative frameworks; theme system compatible with OpenAI/ChatKit integration.
**Testing Requirements**: ✅ Will include accessibility testing (axe-core), responsive design testing, contrast ratio verification, keyboard navigation tests.
**Code Review**: ✅ PRs will verify spec compliance, technical accuracy (Docusaurus docs), reproducibility on clean Docusaurus installs.

**Status**: ✅ PASS - No violations. Proceed to Phase 0 research.

## Project Structure

### Documentation (this feature)

```text
specs/005-docusaurus-ui-upgrade/
├── spec.md                          # Feature specification (complete, 20/20 checklist)
├── plan.md                          # This file (/sp.plan output)
├── research.md                      # Phase 0 output (design patterns, Docusaurus theming, accessibility)
├── data-model.md                    # Phase 1 output (UI component entities, theme config schema)
├── quickstart.md                    # Phase 1 output (theme customization guide for maintainers)
├── contracts/                       # Phase 1 output (component interface specs)
│   ├── design-system.contract.md    # Color, typography, spacing tokens
│   ├── sidebar-navigation.contract.md
│   ├── breadcrumbs.contract.md
│   └── theme-config.contract.md
├── checklists/
│   └── requirements.md              # Quality validation (100% pass)
└── tasks.md                         # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (Docusaurus site root)

```text
docs/                                 # Markdown documentation (no changes)
├── module-1-humanoid-control/
├── module-2-perception-slam/
├── module-3-isaac-brain/
└── module-4-vla/

docusaurus.config.js                 # Theme config injected with design tokens
src/css/
├── custom.css                       # Global styles, CSS variables, responsive breakpoints
├── theme/
│   ├── light.css                    # Light theme token values
│   ├── dark.css                     # Dark theme token values
│   ├── typography.css               # Font families, sizes, weights, line heights
│   ├── spacing.css                  # Spacing scale (8px base units)
│   └── accessibility.css            # Focus states, high contrast support, reduced motion
└── components/
    ├── sidebar.css                  # Module-wise sidebar styling
    ├── breadcrumbs.css              # Breadcrumb navigation
    ├── code-blocks.css              # Code block styling with line height
    └── responsive.css               # Mobile-first breakpoints (320px, 768px, 1024px, 1440px+)

src/swizzled/                        # Swizzled Docusaurus components (React)
├── Root.tsx                         # App root with theme provider
├── Layout/                          # Main layout wrapper
├── Sidebar/                         # Custom sidebar with module expansion
├── Breadcrumbs/                     # Navigation breadcrumbs
├── CodeBlock/                       # Code block styling
└── SearchBar/                       # Enhanced search with module context

tests/
├── accessibility/
│   ├── contrast.test.ts             # WCAG AA contrast ratio verification
│   ├── keyboard-nav.test.ts         # Keyboard navigation testing
│   └── screen-reader.test.ts        # Screen reader compatibility
├── responsive/
│   ├── breakpoints.test.ts          # Responsive design at 4 breakpoints
│   ├── touch-targets.test.ts        # 44x44px minimum touch target validation
│   └── mobile-menu.test.ts          # Mobile hamburger menu interactions
├── theming/
│   ├── dark-mode.test.ts            # Dark/light theme switching
│   ├── css-variables.test.ts        # CSS variable injection verification
│   └── theme-persistence.test.ts    # localStorage persistence
└── performance/
    ├── lighthouse.test.ts           # Lighthouse score validation (≥85)
    └── bundle-size.test.ts          # Performance impact (<20%)
```

**Structure Decision**: Docusaurus site with CSS-based theming (no custom build) using:
- CSS custom properties (variables) for theme tokens
- CSS modules for component scoping
- Docusaurus theme swizzling for component customization
- Mobile-first responsive design approach
- Layer-based architecture: tokens → theme → components → layout

## Complexity Tracking

**No constitutional violations to justify.** All decisions align with spec-first workflow, technical accuracy standards, and Docusaurus stack fidelity.

---

## Phase 0: Research & Technical Foundations

**Duration**: ~2-3 days | **Deliverable**: research.md with findings, decisions, and implementation patterns

### Research Tasks

1. **Docusaurus Theming Architecture**
   - Official docs: Docusaurus theme swizzling, CSS-in-JS modules, theme provider
   - Find: Official examples of CSS variable injection via theme config
   - Outcome: Document approach for custom theme without ejecting

2. **CSS Variables & Light/Dark Mode Implementation**
   - Research: CSS custom properties browser support (caniuse.com)
   - Find: Best practices for theme persistence (localStorage), system preference detection
   - Outcome: Define CSS variable naming convention and theme switching logic

3. **WCAG AA Accessibility Standards**
   - Source: W3C WCAG 2.1 Level AA guidelines
   - Focus: Color contrast (4.5:1), keyboard navigation, focus indicators, semantic HTML
   - Tools: axe-core, WAVE, color contrast checker
   - Outcome: Checklist of a11y requirements and testing approach

4. **Responsive Design Patterns**
   - Research: Mobile-first CSS, CSS media queries, flexible layouts
   - Focus: Breakpoints (320px, 768px, 1024px, 1440px+), touch targets (44x44px minimum)
   - Tools: Viewport meta tag, device emulation, responsive testing frameworks
   - Outcome: Breakpoint naming convention and responsive component patterns

5. **Typography System Design**
   - Research: Modular scale typography, line height best practices
   - Focus: Font families (Docusaurus defaults: Segoe UI, or custom), size scale, weights
   - Outcome: Typography specification with scale and recommended line heights

6. **Design Tokens & Token Management**
   - Research: Design token architectures (naming, organization, generation)
   - Find: Tools for CSS variable organization and theme switching
   - Outcome: Token schema and organization structure

7. **Performance Optimization for Static Sites**
   - Research: Docusaurus build optimization, CSS minification, critical CSS
   - Focus: Meeting <20% performance impact constraint
   - Outcome: Performance baseline and optimization strategies

### Research Outputs

**research.md** will document:
- **Docusaurus Theming Decision**: Use CSS modules + custom theme provider injecting CSS variables into `:root`
  - Rationale: No build changes required, standard Docusaurus pattern, maintainable
  - Alternative rejected: Tailwind CSS (adds build complexity)

- **Light/Dark Mode Decision**: CSS variables with system preference detection + localStorage override
  - Rationale: Performant, no flash on page load, respects user preference

- **Typography Decision**: Use system fonts (Segoe UI, San Francisco) with fallbacks for performance
  - Rationale: Zero custom font downloads, excellent performance, professional appearance

- **WCAG AA Approach**: Enforce via axe-core in tests, provide checklist for manual review

- **Responsive Breakpoints**: Mobile (320px), Tablet (768px), Desktop (1024px), Large (1440px+)

- **CSS Variable Naming**: `--ds-color-primary`, `--ds-spacing-base`, `--ds-font-size-body`
  - Convention: `--ds-category-variant` for consistency

---

## Phase 1: Design & Architecture

**Duration**: ~3-4 days | **Deliverables**: data-model.md, contracts/, quickstart.md, updated plan.md

### Design Tasks

#### 1. Data Model Design (data-model.md)
Define entities and relationships:

**Theme Configuration Entity**
```
ThemeConfig {
  colors: {
    light: { primary, secondary, accent, neutral, success, warning, error },
    dark: { ... same keys with adjusted values ... }
  },
  typography: {
    fonts: { heading, body, code },
    sizes: { xs, sm, base, lg, xl, 2xl, 3xl, 4xl },
    weights: { normal, semibold, bold },
    lineHeights: { tight, normal, relaxed }
  },
  spacing: {
    base: "8px",
    scale: [0, 8, 16, 24, 32, 48, 64, 96] // 8px increments
  },
  breakpoints: {
    mobile: "320px",
    tablet: "768px",
    desktop: "1024px",
    large: "1440px"
  }
}
```

**UI Component Entities**
```
Sidebar {
  modules: Module[],
  expandedModules: string[], // module IDs
  currentPage: string // document ID
}

Module {
  id: string,
  name: string, // "Humanoid Control", etc.
  chapters: Chapter[],
  expanded: boolean
}

Chapter {
  id: string,
  title: string,
  documentId: string, // for routing
  highlighted: boolean // current page
}

Breadcrumb {
  items: BreadcrumbItem[]
}

BreadcrumbItem {
  label: string,
  href: string,
  isActive: boolean
}

UserThemePreference {
  theme: "light" | "dark",
  persistedAt: timestamp,
  source: "user-click" | "system-preference"
}
```

#### 2. Component Contracts (contracts/)

**design-system.contract.md**: Token definitions
- Color token values (light/dark), contrast ratios verified
- Typography scale with line heights
- Spacing scale
- Breakpoint definitions

**sidebar-navigation.contract.md**: Props and interactions
```
Props: {
  modules: Module[],
  currentPageId: string,
  onNavigate: (docId: string) => void,
  onToggleModule: (moduleId: string) => void
}
Events: {
  expand-module, collapse-module, navigate-page, focus-item
}
Accessibility: {
  role: "navigation",
  aria-label: "Documentation sidebar",
  keyboard: Tab (focus), Enter (expand/navigate), Escape (close on mobile)
}
```

**breadcrumbs.contract.md**: Navigation context
```
Props: {
  items: BreadcrumbItem[],
  separator: string
}
Accessibility: {
  role: "navigation",
  aria-label: "Breadcrumb",
  aria-current: "page" on final item
}
```

**theme-config.contract.md**: Configuration schema
```
ThemeConfig {
  colors: { light: {...}, dark: {...} },
  typography: { fonts, sizes, weights, lineHeights },
  spacing: { base, scale },
  breakpoints: { mobile, tablet, desktop, large }
}
```

#### 3. Quickstart Guide (quickstart.md)
For site maintainers:
- How to customize colors via CSS variables
- How to change fonts
- How to adjust spacing
- How to add dark mode to new components
- Examples with before/after code

#### 4. Architectural Decisions
- **Decision 1: CSS Variables vs. SCSS/Tailwind**
  - Choice: CSS Variables
  - Rationale: No build changes, Docusaurus compatible, maintainers can edit colors without dev

- **Decision 2: Swizzling vs. Custom Components**
  - Choice: Swizzle critical components (Sidebar, Layout, Root) + custom CSS
  - Rationale: Minimal code duplication, upgradeable with Docusaurus

- **Decision 3: Responsive Approach**
  - Choice: Mobile-first CSS (start at 320px, add media queries for larger)
  - Rationale: Better performance, cleaner CSS, prioritizes mobile UX

- **Decision 4: Dark Mode Persistence**
  - Choice: localStorage with system preference fallback
  - Rationale: Respects user choice, no server needed, fast

### Evaluation
- Constitution Check: Re-verify spec-first (✅), technical accuracy (✅), reproducibility (✅)
- Gates: All pass. Proceed to task generation.

---

## Phase 2: Task Generation & Implementation Planning

**Duration**: Handled by `/sp.tasks` command | **Output**: tasks.md (granular, dependency-ordered)

Task breakdown (estimated 50-80 tasks across phases):

### Phase 2A: Design System Setup (8-12 tasks)
- Create CSS variable definitions (colors, typography, spacing)
- Set up light/dark theme CSS files
- Define responsive breakpoint variables
- Create custom.css base styles
- Verify WCAG AA contrast ratios

### Phase 2B: Core Layout & Navigation (12-16 tasks)
- Swizzle Root component for theme provider
- Swizzle Layout component for responsive grid
- Create custom Sidebar component with module expansion
- Create Breadcrumbs component
- Implement mobile hamburger menu
- Add focus indicators and keyboard navigation

### Phase 2C: Responsive Design (10-14 tasks)
- Style components at mobile breakpoint (320px)
- Add tablet styles (768px)
- Add desktop styles (1024px)
- Add large screen styles (1440px+)
- Verify 44x44px touch targets
- Test on actual devices

### Phase 2D: Dark Mode & Theming (6-10 tasks)
- Implement theme toggle component
- Set up localStorage persistence
- Detect system preference (prefers-color-scheme)
- Apply dark theme CSS variables
- Verify contrast on dark theme

### Phase 2E: Accessibility & Testing (8-12 tasks)
- Add ARIA labels to components
- Implement keyboard navigation (Tab, Enter, Escape)
- Test with screen readers
- Run axe-core accessibility scans
- Verify color contrast with automated tools
- Manual accessibility review

### Phase 2F: Performance & Optimization (6-8 tasks)
- Measure baseline Lighthouse score
- Optimize critical CSS
- Minimize bundle impact
- Verify <20% performance impact
- Optimize images and code blocks

### Phase 2G: Search Enhancement (4-6 tasks)
- Add module/chapter metadata to search index
- Enhance search results with context
- Test search ranking by module

### Phase 2H: Documentation & QA (6-10 tasks)
- Write theme customization guide (quickstart.md)
- Document component interfaces (contracts/)
- Create contributor guide for styling
- Test on Chrome, Firefox, Safari, Edge
- Conduct user feedback session

---

## Phase 3: Production Deployment

**Timing**: After all Phase 2 tasks complete and tests pass

**Checklist**:
- ✅ All FRs implemented and tested
- ✅ WCAG AA accessibility verified
- ✅ Responsive on 320px-1440px+
- ✅ Dark/light theme with persistence
- ✅ Performance within budget (<20% slower)
- ✅ Cross-browser compatibility verified
- ✅ All tests passing (unit, e2e, accessibility, performance)
- ✅ PR reviewed and merged to main
- ✅ Docusaurus build succeeds with [SUCCESS]
- ✅ Static site deploys without errors
- ✅ User feedback gathered (≥90% approve design)

---

## Key Decisions & Rationale

| Decision | Rationale | Impact |
|----------|-----------|--------|
| **CSS Variables** | No build changes, maintainer-friendly | Simple, extensible theming |
| **Mobile-First Design** | Better performance, mobile-focused | Clean CSS, good UX on all devices |
| **Swizzled Components** | Minimal code, upgradeable | Maintainable, reduced duplication |
| **System Font Stack** | Zero download overhead | Excellent performance baseline |
| **localStorage Persistence** | Fast, no server required | Instant dark mode on return visit |
| **4 Breakpoints** | Cover device range (mobile-desktop-large) | Universal support |
| **WCAG AA Target** | Inclusive, measurable, auditable | Accessible to all users |

---

## Success Criteria Validation

**All 15 success criteria from spec are addressable by this architecture**:

| SC | Target | Architecture Support |
|----|--------|----------------------|
| Modules in sidebar | 100% visible | Custom Sidebar component + module-wise data model |
| WCAG AA contrast | 4.5:1 verified | CSS variables with verified values + axe-core tests |
| Responsive all sizes | 320px-1440px+ | 4-breakpoint mobile-first CSS |
| User satisfaction | 90% approve | User feedback session + design validation |
| Dark mode persistent | localStorage | Dark theme CSS + localStorage logic in Root component |
| <20% perf impact | Measured via Lighthouse | Baseline + optimization tasks |
| Keyboard accessible | 100% interactive | Tab, Enter, Escape implemented + tested |
| Customizable config | CSS variables | Theme config JSON + CSS variable injection |
| Component consistency | 100% styled | CSS modules scope, design system enforces consistency |
| Browser compat | Latest 2 versions | Test on Chrome, Firefox, Safari, Edge |
| Touch targets | 44x44px minimum | Design system + responsive testing |
| Search context | Module/chapter ranked | Search enhancement tasks |
| Typography system | Unified fonts/sizes | Design system tokens + component styling |
| Print quality | Full content, no breaks | Print CSS media queries |
| Load time | <20% increase | Performance baseline + optimization |

---

## Risk Analysis & Mitigation

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|-----------|
| Docusaurus API changes break swizzling | Low | Medium | Use stable Docusaurus 2.x LTS; test against latest before upgrade |
| CSS variable browser compat issues | Very Low | Medium | caniuse.com shows 98%+ support; fallbacks for older browsers |
| Dark mode causes contrast issues | Medium | Medium | Axe-core tests + manual verification of all dark theme colors |
| Responsive design breaks on uncommon devices | Low | Low | Test with Chrome DevTools, actual devices, user feedback |
| Performance regression from styling | Low | Medium | Lighthouse baseline before/after; optimize critical CSS |

---

## Acceptance & Next Steps

**Plan Status**: ✅ **COMPLETE** - Ready for task generation via `/sp.tasks`

**Next Command**: `/sp.tasks` to generate:
- Granular, dependency-ordered implementation tasks (50-80 estimated)
- Task breakdown by phase (2A-2H + Phase 3)
- Acceptance criteria for each task
- Test cases for verification

**Commit Plan**: All plan artifacts (plan.md, research.md, data-model.md, contracts/, quickstart.md) will be committed in a single PR after `/sp.tasks` completes.

**Success Metrics Post-Implementation**:
- 100% of 24 FRs implemented
- All 15 success criteria met
- 20/20 tests passing (accessibility, responsive, performance, browser compat)
- 90%+ user satisfaction on design
- <20% performance impact vs. baseline
