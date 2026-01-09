# Tasks: 005-Docusaurus UI Upgrade with Modern Design

**Input**: Design documents from `/specs/005-docusaurus-ui-upgrade/`
**References**: plan.md, spec.md, research.md, data-model.md, quickstart.md, contracts/

**Feature Summary**: Modernize Docusaurus documentation site with clean UI, responsive design (320px-1440px+), WCAG AA compliance, dark/light theming via CSS variables, and module-wise navigation. All content preserved, only design and UX enhanced.

**Tests**: Tests are OPTIONAL and specified per phase based on feature requirements. All test tasks are included where verification is critical.

**Organization**: Tasks grouped by user story (5 user stories from spec) to enable independent implementation and testing. Design system setup and responsive design are foundational (Phase 2 - must complete before user stories).

## Format: `- [ ] [ID] [P?] [Story?] Description with file path`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story (US1-US5) - omitted for Setup/Foundational/Polish phases
- **File paths**: Absolute from repository root

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and asset creation

- [ ] T001 Create CSS directory structure at `src/css/` with subdirectories: `theme/`, `components/`
- [ ] T002 Create swizzled components directory at `src/swizzled/` for Root, Layout, Sidebar, Breadcrumbs
- [ ] T003 Create tests directory structure at `tests/` with: `accessibility/`, `responsive/`, `theming/`, `performance/`
- [ ] T004 [P] Initialize docusaurus.config.js with custom CSS injection configuration in `docusaurus.config.js`
- [ ] T005 [P] Copy existing Docusaurus theme files for safe swizzling into `src/swizzled/`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core design system and responsive infrastructure that MUST complete before ANY user story

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### Design System Setup (CSS Variables & Tokens)

- [ ] T006 Create design system contract file with all token definitions in `specs/005-docusaurus-ui-upgrade/contracts/design-system.contract.md`
- [ ] T007 [P] Create light theme CSS variables in `src/css/theme/light.css` with colors, typography, spacing, breakpoints
- [ ] T008 [P] Create dark theme CSS variables in `src/css/theme/dark.css` with adjusted colors for dark mode
- [ ] T009 [P] Create typography CSS system in `src/css/theme/typography.css` with font families (system fonts), sizes (xs-4xl), weights, line heights
- [ ] T010 [P] Create spacing scale in `src/css/theme/spacing.css` with 8px base unit scale (0, 4, 8, 12, 16, 24, 32, 48, 64, 96, 128)
- [ ] T011 [P] Create responsive breakpoints in `src/css/theme/responsive.css` for mobile (320px), tablet (768px), desktop (1024px), large (1440px)
- [ ] T012 [P] Create accessibility CSS in `src/css/theme/accessibility.css` with focus states, reduced motion support, high contrast utilities
- [ ] T013 Merge all theme files into unified base in `src/css/custom.css` with `:root` (light) and `[data-theme="dark"]` overrides

### Contrast Verification (WCAG AA)

- [ ] T014 Verify all light theme text colors meet 4.5:1 contrast ratio in `src/css/theme/light.css` (use WebAIM checker)
- [ ] T015 Verify all dark theme text colors meet 4.5:1 contrast ratio in `src/css/theme/dark.css` (verify both themes)
- [ ] T016 Create accessibility checklist in `specs/005-docusaurus-ui-upgrade/contracts/accessibility-checklist.md` with WCAG AA requirements

### Responsive Design Foundation

- [ ] T017 [P] Create mobile-first responsive utilities in `src/css/responsive.css` for breakpoint-based layout adjustments
- [ ] T018 [P] Define touch target sizing (44x44px minimum) CSS in `src/css/components/responsive.css` for all interactive elements
- [ ] T019 Create responsive design testing guide in `docs/responsive-design-testing.md` with breakpoint test plan

### Theme Persistence Infrastructure

- [ ] T020 Create Root component wrapper in `src/swizzled/Root.tsx` with theme context provider and localStorage detection
- [ ] T021 Implement theme toggle logic in `src/swizzled/Root.tsx`: read localStorage, detect system preference, apply CSS variables
- [ ] T022 Create theme persistence script in `src/css/custom.css` inline `<script>` to prevent FOUC (flash of unstyled content)

**Checkpoint**: Design system complete - all user stories can now proceed in parallel

---

## Phase 3: User Story 1 - Module Discovery & Navigation (Priority: P1) 🎯 MVP

**Goal**: Readers understand 4-module structure and navigate to chapters quickly via organized sidebar

**Independent Test**: Open site, verify all 4 modules visible in sidebar with all chapters, click module to expand/collapse, navigate between pages using sidebar

### Implementation for User Story 1

- [ ] T023 [P] [US1] Create Sidebar component in `src/swizzled/Sidebar/index.tsx` with module/chapter structure, expand/collapse handlers
- [ ] T024 [P] [US1] Create sidebar stylesheet in `src/css/components/sidebar.css` with module list styling, hover states, active page highlight
- [ ] T025 [P] [US1] Create custom Layout wrapper in `src/swizzled/Layout/index.tsx` that includes Sidebar + main content grid
- [ ] T026 [US1] Implement module data structure parsing in `src/swizzled/Sidebar/index.tsx` from docusaurus.config.js sidebars
- [ ] T027 [US1] Add keyboard navigation to Sidebar in `src/swizzled/Sidebar/index.tsx`: Tab (focus), Enter (expand/navigate), Escape (close on mobile)
- [ ] T028 [US1] Add ARIA labels to Sidebar in `src/swizzled/Sidebar/index.tsx`: role="navigation", aria-label="Documentation sidebar", aria-expanded state
- [ ] T029 [US1] Implement mobile hamburger menu toggle in `src/swizzled/Layout/index.tsx` with responsive visibility (show mobile, hide tablet+)
- [ ] T030 [US1] Add focus indicators to all sidebar items in `src/css/components/sidebar.css` with visible outline on Tab navigation

### Tests for User Story 1

- [ ] T031 [P] [US1] Accessibility test for sidebar keyboard navigation in `tests/accessibility/keyboard-nav.test.ts`
- [ ] T032 [P] [US1] Responsive test for sidebar at 4 breakpoints in `tests/responsive/sidebar.test.ts` (320px, 768px, 1024px, 1440px)
- [ ] T033 [US1] Verify all modules visible and expandable in `tests/integration/sidebar-navigation.test.ts`

**Checkpoint**: User Story 1 complete - sidebar navigation fully functional and testable independently

---

## Phase 4: User Story 2 - Modern Visual Design (Priority: P1)

**Goal**: Readers enjoy clean interface with excellent typography, contrast, and spacing

**Independent Test**: Open site on desktop and mobile, verify clean visual appearance with consistent spacing, readable typography, proper contrast on light and dark themes

### Implementation for User Story 2

- [ ] T034 [P] [US2] Create breadcrumbs component in `src/swizzled/Breadcrumbs/index.tsx` with navigation trail (Home > Module > Chapter)
- [ ] T035 [P] [US2] Create breadcrumbs stylesheet in `src/css/components/breadcrumbs.css` with spacing, separators, active page styling
- [ ] T036 [P] [US2] Update main content layout in `src/swizzled/Layout/index.tsx` to use responsive grid (sidebar + content)
- [ ] T037 [P] [US2] Create code block styling in `src/css/components/code-blocks.css` with proper font family, size, line height, contrast
- [ ] T038 [US2] Add ARIA labels to breadcrumbs in `src/swizzled/Breadcrumbs/index.tsx`: role="navigation", aria-current="page" on active
- [ ] T039 [US2] Style headers (h1-h6) with typography scale in `src/css/theme/typography.css`: proper sizes, weights, line heights
- [ ] T040 [US2] Style body text and paragraphs in `src/css/custom.css` with line-height 1.6, font-size base (16px)
- [ ] T041 [US2] Create consistent link styling in `src/css/custom.css`: color, hover state, focus indicator, underline
- [ ] T042 [US2] Update list styles (ul, ol) in `src/css/custom.css` with proper indentation, spacing, marker styles
- [ ] T043 [US2] Create button/interactive element styling in `src/css/custom.css` with padding, borders, focus indicators, touch targets ≥44x44px
- [ ] T044 [US2] Add visual hierarchy through color, contrast, spacing in `src/css/custom.css` (primary color: #0066cc, secondary: #6c757d)

### Tests for User Story 2

- [ ] T045 [P] [US2] Visual regression test for typography system in `tests/design/typography.test.ts`
- [ ] T046 [P] [US2] Contrast ratio verification test in `tests/accessibility/contrast.test.ts` (4.5:1 minimum for all text)
- [ ] T047 [US2] Responsive spacing test in `tests/responsive/spacing.test.ts` (verify consistent spacing at all breakpoints)
- [ ] T048 [US2] Screen reader test for breadcrumbs in `tests/accessibility/screen-reader.test.ts`

**Checkpoint**: User Stories 1 & 2 complete - clean, visually consistent design with proper hierarchy and accessibility

---

## Phase 5: User Story 3 - Customizable Theme (Priority: P2)

**Goal**: Site maintainers customize colors/fonts via config without editing content

**Independent Test**: Edit CSS variables in `src/css/custom.css`, refresh browser, verify changes apply to all pages (colors, fonts, spacing)

### Implementation for User Story 3

- [ ] T049 [P] [US3] Create theme configuration schema in `src/css/custom.css` with documented CSS variable naming convention
- [ ] T050 [P] [US3] Document color customization in `specs/005-docusaurus-ui-upgrade/quickstart.md` with before/after examples
- [ ] T051 [P] [US3] Document typography customization in `specs/005-docusaurus-ui-upgrade/quickstart.md` (font families, sizes, weights)
- [ ] T052 [P] [US3] Document spacing customization in `specs/005-docusaurus-ui-upgrade/quickstart.md` (base unit changes, scale adjustments)
- [ ] T053 [US3] Create theme config validation helper in `scripts/validate-theme-config.js` (optional - ensures valid hex colors, spacing values)
- [ ] T054 [US3] Create theme customization examples in `specs/005-docusaurus-ui-upgrade/quickstart.md`: brand color, high contrast, spacious layout, larger text

### Tests for User Story 3

- [ ] T055 [P] [US3] CSS variable injection test in `tests/theming/css-variables.test.ts` (verify variables present in :root)
- [ ] T056 [P] [US3] Theme customization example verification in `tests/theming/customization.test.ts` (sample color change applies)

**Checkpoint**: User Stories 1, 2, & 3 complete - clean design fully customizable by maintainers

---

## Phase 6: User Story 4 - Responsive Design (Priority: P2)

**Goal**: Mobile users experience optimized layout and touch-friendly navigation

**Independent Test**: Test site on mobile (320px), tablet (768px), desktop (1024px), large (1440px) - verify layout adjustments, touch targets ≥44x44px, hamburger menu on mobile

### Implementation for User Story 4

- [ ] T057 [P] [US4] Create mobile-specific Layout in `src/swizzled/Layout/index.tsx` with hamburger menu, full-width content at 320px
- [ ] T058 [P] [US4] Style mobile navigation in `src/css/components/sidebar.css`: hamburger button placement, menu overlay, close button
- [ ] T059 [P] [US4] Create tablet styles in `src/css/custom.css` @media (min-width: 768px): collapsible sidebar, 2-column layout
- [ ] T060 [P] [US4] Create desktop styles in `src/css/custom.css` @media (min-width: 1024px): full sidebar, 3-column layout for wide screens
- [ ] T061 [P] [US4] Create large screen styles in `src/css/custom.css` @media (min-width: 1440px): max-width constraints, wide sidebar
- [ ] T062 [US4] Verify touch targets in all components: buttons, links, sidebar items, breadcrumbs all ≥44x44px (test in `tests/responsive/touch-targets.test.ts`)
- [ ] T063 [US4] Create mobile menu auto-close logic in `src/swizzled/Layout/index.tsx`: close menu on page navigation, resize to desktop
- [ ] T064 [US4] Add viewport meta tag support in `src/swizzled/Root.tsx` (already in Docusaurus, verify: width=device-width, initial-scale=1)
- [ ] T065 [US4] Style images for responsiveness in `src/css/custom.css`: max-width: 100%, height: auto
- [ ] T066 [US4] Create print styles in `src/css/custom.css` @media print: no navigation, full content, readable fonts

### Tests for User Story 4

- [ ] T067 [P] [US4] Responsive breakpoint test in `tests/responsive/breakpoints.test.ts` (verify layout at 320px, 768px, 1024px, 1440px)
- [ ] T068 [P] [US4] Touch target sizing test in `tests/responsive/touch-targets.test.ts` (all interactive elements ≥44x44px)
- [ ] T069 [P] [US4] Mobile menu interaction test in `tests/responsive/mobile-menu.test.ts` (hamburger opens/closes, auto-closes on nav)
- [ ] T070 [US4] Responsive image test in `tests/responsive/images.test.ts` (images scale properly at all breakpoints)

**Checkpoint**: User Stories 1-4 complete - fully responsive design across all device sizes

---

## Phase 7: User Story 5 - Quick Content Search (Priority: P3)

**Goal**: Readers find content via search with module context

**Independent Test**: Use search bar, verify results include module/chapter context, results are ranked by relevance with module information

### Implementation for User Story 5

- [ ] T071 [P] [US5] Enhance search metadata in docusaurus.config.js: add module, chapter fields to indexed documents
- [ ] T072 [P] [US5] Create search context provider in `src/swizzled/SearchBar/index.tsx` if custom search needed
- [ ] T073 [US5] Modify search results presentation to include module context (if using custom search)
- [ ] T074 [US5] Test search functionality with module/chapter context in `tests/integration/search.test.ts`

### Tests for User Story 5

- [ ] T075 [P] [US5] Search results context test in `tests/integration/search.test.ts` (results show module/chapter)

**Checkpoint**: User Stories 1-5 complete - all core features fully functional

---

## Phase 8: Theme Toggle & Dark Mode

**Purpose**: Implement light/dark theme switching with persistence

### Implementation

- [ ] T076 [P] Create theme toggle component in `src/swizzled/ThemeToggle/index.tsx` with icon button, accessibility labels
- [ ] T077 [P] Create theme toggle stylesheet in `src/css/components/theme-toggle.css` with positioning, icon styling, active state
- [ ] T078 Add theme toggle to header/nav in `src/swizzled/Layout/index.tsx` (top right, visible on all pages)
- [ ] T079 Implement localStorage persistence for theme in `src/swizzled/Root.tsx`: save on toggle, restore on page load
- [ ] T080 Implement system preference detection in `src/swizzled/Root.tsx`: prefers-color-scheme media query as fallback
- [ ] T081 Add ARIA labels to theme toggle in `src/swizzled/ThemeToggle/index.tsx`: aria-label, aria-pressed state
- [ ] T082 Create focus indicator for theme toggle in `src/css/components/theme-toggle.css`

### Tests

- [ ] T083 [P] Dark mode application test in `tests/theming/dark-mode.test.ts` (verify CSS variables change with data-theme)
- [ ] T084 [P] Theme persistence test in `tests/theming/theme-persistence.test.ts` (save/load localStorage, system preference fallback)
- [ ] T085 Theme toggle keyboard navigation test in `tests/accessibility/keyboard-nav.test.ts` (Tab, Space/Enter activate toggle)

**Checkpoint**: Theme switching fully functional with persistence and accessibility

---

## Phase 9: Performance & Accessibility Testing

**Purpose**: Verify performance impact <20%, WCAG AA compliance, browser compatibility

### Performance Validation

- [ ] T086 Establish baseline Lighthouse score in `tests/performance/lighthouse.test.ts` (measure before CSS additions)
- [ ] T087 Measure performance impact after all CSS additions in `tests/performance/lighthouse.test.ts` (verify <20% slower)
- [ ] T088 Verify First Contentful Paint <2s in `tests/performance/lighthouse.test.ts`
- [ ] T089 Verify Largest Contentful Paint <2.5s in `tests/performance/lighthouse.test.ts`
- [ ] T090 Verify Cumulative Layout Shift <0.1 in `tests/performance/lighthouse.test.ts`
- [ ] T091 Verify bundle size impact <15KB in `tests/performance/bundle-size.test.ts` (CSS + JS for theming)

### Accessibility Full Verification

- [ ] T092 [P] Run axe-core automated accessibility scan in CI/CD (create `.github/workflows/a11y.yml`)
- [ ] T093 [P] Manual keyboard navigation test in `tests/accessibility/full-keyboard-nav.test.ts`: Tab through all elements, verify focus visible, no keyboard traps
- [ ] T094 [P] Manual screen reader test in `tests/accessibility/full-screen-reader.test.ts`: test with NVDA/JAWS/VoiceOver
- [ ] T095 [P] Color contrast verification in `tests/accessibility/full-contrast.test.ts`: all text 4.5:1 in light and dark themes
- [ ] T096 [P] ARIA completeness check in `tests/accessibility/aria-completeness.test.ts`: all interactive elements have roles, labels, states
- [ ] T097 Manual visual testing on actual devices in `tests/manual/device-testing.md` (iPhone, Android, Windows, macOS)

### Cross-Browser Testing

- [ ] T098 [P] Test on Chrome 120+ in `tests/browser-compat/chrome.test.ts`
- [ ] T099 [P] Test on Firefox 121+ in `tests/browser-compat/firefox.test.ts`
- [ ] T100 [P] Test on Safari 17+ in `tests/browser-compat/safari.test.ts`
- [ ] T101 [P] Test on Edge 120+ in `tests/browser-compat/edge.test.ts`

**Checkpoint**: All accessibility and performance requirements verified

---

## Phase 10: Documentation & Quality Assurance

**Purpose**: Document implementation, verify all requirements met, prepare for launch

### Documentation

- [ ] T102 Update quickstart guide in `specs/005-docusaurus-ui-upgrade/quickstart.md` with final color/font/spacing values
- [ ] T103 Create component interface specs in `specs/005-docusaurus-ui-upgrade/contracts/sidebar-navigation.contract.md`
- [ ] T104 Create component interface specs in `specs/005-docusaurus-ui-upgrade/contracts/breadcrumbs.contract.md`
- [ ] T105 Create component interface specs in `specs/005-docusaurus-ui-upgrade/contracts/theme-config.contract.md`
- [ ] T106 Create contributor guide in `docs/CONTRIBUTOR-STYLING.md` with CSS variables, component styling patterns
- [ ] T107 Create design system usage guide in `docs/design-system-usage.md` with examples of using tokens in components
- [ ] T108 Update main README if needed with design system overview

### Quality Assurance Checklist

- [ ] T109 [P] Verify 100% of 24 functional requirements implemented (check against `specs/005-docusaurus-ui-upgrade/spec.md`)
- [ ] T110 [P] Verify 100% of 15 success criteria met (check against `specs/005-docusaurus-ui-upgrade/plan.md`)
- [ ] T111 [P] Verify 4 modules, all chapters visible in sidebar (test against actual docs structure)
- [ ] T112 [P] Verify user satisfaction criteria: test with 5-10 users, gather feedback on design/usability
- [ ] T113 Run full test suite: `npm test` (all unit, integration, accessibility, performance tests pass)
- [ ] T114 Run Lighthouse on production build: score ≥85, FCP <2s, no violations
- [ ] T115 Run axe-core on production build: zero violations
- [ ] T116 Verify Docusaurus build succeeds: `npm run build` produces error-free output

### Final QA Tasks

- [ ] T117 Create implementation summary in `IMPLEMENTATION_SUMMARY.md` documenting all changes
- [ ] T118 Create requirements checklist validation in `specs/005-docusaurus-ui-upgrade/requirements-checklist.md` (mark each FR/SC as complete)
- [ ] T119 Test on real mobile devices: iPhone (Safari), Android (Chrome) - verify touch interactions, visibility, responsiveness

**Checkpoint**: All requirements verified, documentation complete, ready for launch

---

## Phase 11: Polish & Cross-Cutting Concerns

**Purpose**: Final improvements and refinements across all features

- [ ] T120 [P] Code cleanup: remove unused CSS, commented code, debug statements
- [ ] T121 [P] CSS organization review: ensure consistent formatting, comment sections, logical grouping
- [ ] T122 Review component code for consistency: naming conventions, prop patterns, accessibility completeness
- [ ] T123 Add missing error handling or edge cases discovered during testing
- [ ] T124 Optimize CSS: remove duplicate rules, consolidate similar selectors (if minification doesn't catch)
- [ ] T125 Performance optimization pass: verify no layout thrashing, unnecessary reflows, slow selectors
- [ ] T126 Accessibility polish: ensure all focus indicators are clear, all keyboard shortcuts documented
- [ ] T127 Create PHR (Prompt History Record) for implementation workflow in `history/prompts/005-docusaurus-ui-upgrade/`

**Checkpoint**: All code polished, ready for merge to main

---

## Phase 12: Deployment & Launch

**Purpose**: Merge, deploy, and monitor production

- [ ] T128 [P] Code review: PR reviewed for spec compliance, technical accuracy, accessibility
- [ ] T129 [P] Pre-merge verification: all tests passing, Lighthouse ≥85, axe-core zero violations
- [ ] T130 Merge to main branch with comprehensive commit message
- [ ] T131 Deploy to production (via CI/CD)
- [ ] T132 Monitor production for 24 hours: check error logs, user feedback, performance metrics
- [ ] T133 Create post-launch summary in `PHASE-1-COMPLETION-REPORT.md` with metrics, user feedback, learnings

**Checkpoint**: Feature live in production, ready for user feedback

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies - can start immediately
- **Phase 2 (Foundational)**: Depends on Phase 1 - **BLOCKS all user stories**
- **Phases 3-7 (User Stories 1-5)**: Depend on Phase 2 - can proceed in parallel or sequentially
- **Phase 8 (Theme Toggle)**: Depends on Phase 2 (design system) but can start after Phase 3
- **Phase 9 (Performance & Accessibility)**: Depends on Phases 3-8 - comprehensive testing
- **Phase 10 (Documentation & QA)**: Depends on Phases 3-9 - final validation
- **Phase 11 (Polish)**: Depends on Phase 10 - final refinements
- **Phase 12 (Deployment)**: Depends on Phase 11 - production launch

### User Story Dependencies

- **US1 (Module Navigation)**: Can start after Phase 2 - No dependencies on other stories
- **US2 (Visual Design)**: Can start after Phase 2 - No dependencies on other stories (uses same design system)
- **US3 (Customizable Theme)**: Can start after Phase 2 - Depends on US1/US2 being styled but independent implementation
- **US4 (Responsive Design)**: Can start after Phase 2 - Affects all user stories but independent testing
- **US5 (Search)**: Can start after Phase 2 - No dependencies on other stories

### Within Each User Story

- Implementation before tests (or tests-first if TDD preferred)
- Models/structure before styling
- Core implementation before integration

### Parallel Opportunities

- **Phase 1 Tasks**: All marked [P] can run in parallel (T004, T005)
- **Phase 2 Tasks**: CSS theme tasks marked [P] can run in parallel (T007-T012)
- **Phase 3-7 Tasks**: Component implementation marked [P] can run in parallel (different files)
- **Phase 9 Tests**: All test categories marked [P] can run in parallel (accessibility, performance, browser compat)
- **Different User Stories**: Can be worked on in parallel by different team members once Phase 2 completes

---

## Parallel Execution Example: Full Team

**Timeline**: With 3 developers after Phase 2 complete:

```
Developer A: User Story 1 (Sidebar Navigation)      [T023-T033]
Developer B: User Story 2 (Visual Design)           [T034-T048]
Developer C: User Story 3 (Customizable Theme)      [T049-T056]

Then collectively:
Phase 8: Theme Toggle                               [T076-T085]
Phase 9: Performance & Accessibility Testing        [T086-T101]
Phase 10: Documentation & QA                        [T102-T119]
Phase 11: Polish                                    [T120-T127]
Phase 12: Deployment                                [T128-T133]
```

---

## MVP Strategy (User Story 1 Only)

For a minimal viable product with just sidebar navigation:

1. ✅ Complete Phase 1: Setup (5 tasks)
2. ✅ Complete Phase 2: Foundational (15 tasks)
3. ✅ Complete Phase 3: User Story 1 (11 tasks)
4. ✅ Complete Phase 8: Theme Toggle (7 tasks - users can see dark/light)
5. ✅ Complete Phase 9: Testing (partial - focus on accessibility, performance)
6. ✅ Complete Phase 12: Deploy
7. **Total for MVP**: ~40-50 tasks, ~2 weeks with 1-2 developers

Then incrementally add US2-US5 in following sprints.

---

## Implementation Notes

### Critical Path

1. **T001-T005**: Setup (foundational, fast)
2. **T006-T022**: Design system (blocking - do NOT skip)
3. **T023-T033**: Sidebar (validates design system works)
4. **T034-T085**: Remaining features (can parallelize after sidebar validated)

### File Organization Convention

```
src/
├── css/
│   ├── custom.css              # Imports all themes, main entry
│   ├── theme/
│   │   ├── light.css           # Light theme variables
│   │   ├── dark.css            # Dark theme variables
│   │   ├── typography.css      # Font definitions
│   │   ├── spacing.css         # Spacing scale
│   │   ├── responsive.css      # Breakpoints
│   │   └── accessibility.css   # Focus, reduced motion
│   └── components/
│       ├── sidebar.css         # Sidebar styling
│       ├── breadcrumbs.css     # Breadcrumb styling
│       ├── code-blocks.css     # Code block styling
│       ├── theme-toggle.css    # Theme toggle styling
│       └── responsive.css      # Responsive utilities
└── swizzled/
    ├── Root.tsx                # Theme context, persistence
    ├── Layout/
    │   └── index.tsx           # Main layout, sidebar + content
    ├── Sidebar/
    │   └── index.tsx           # Module/chapter navigation
    ├── Breadcrumbs/
    │   └── index.tsx           # Navigation breadcrumbs
    ├── ThemeToggle/
    │   └── index.tsx           # Dark/light toggle button
    └── SearchBar/
        └── index.tsx           # Enhanced search (if custom)

tests/
├── accessibility/
│   ├── keyboard-nav.test.ts
│   ├── screen-reader.test.ts
│   ├── contrast.test.ts
│   └── aria-completeness.test.ts
├── responsive/
│   ├── breakpoints.test.ts
│   ├── touch-targets.test.ts
│   ├── mobile-menu.test.ts
│   └── images.test.ts
├── theming/
│   ├── dark-mode.test.ts
│   ├── css-variables.test.ts
│   ├── theme-persistence.test.ts
│   └── customization.test.ts
├── performance/
│   ├── lighthouse.test.ts
│   └── bundle-size.test.ts
├── browser-compat/
│   ├── chrome.test.ts
│   ├── firefox.test.ts
│   ├── safari.test.ts
│   └── edge.test.ts
└── integration/
    ├── sidebar-navigation.test.ts
    ├── search.test.ts
    └── full-keyboard-nav.test.ts

specs/005-docusaurus-ui-upgrade/
├── spec.md                          # Requirements
├── plan.md                          # Architecture
├── research.md                      # Technical decisions
├── data-model.md                    # Entities
├── quickstart.md                    # Customization guide
├── contracts/
│   ├── design-system.contract.md
│   ├── sidebar-navigation.contract.md
│   ├── breadcrumbs.contract.md
│   ├── theme-config.contract.md
│   └── accessibility-checklist.md
├── requirements-checklist.md        # Validation
└── tasks.md                         # This file
```

### Success Criteria Mapping

| Success Criteria | Task(s) | Verification |
|------------------|---------|--------------|
| 100% modules/chapters in sidebar | T023-T026 | T031-T033, T111 |
| WCAG AA contrast (4.5:1) | T014-T016 | T046, T095 |
| Responsive 320px-1440px+ | T057-T070 | T067-T070, T097 |
| 90% user satisfaction | T112 | User feedback session |
| Dark mode with persistence | T076-T082 | T083-T085 |
| Customizable via config | T049-T054 | T055-T056 |
| Keyboard accessible (100%) | T027-T028, T038, T082 | T093-T094 |
| <20% performance impact | T086-T091 | Lighthouse ≥85 |
| Touch targets ≥44x44px | T018, T062 | T068 |
| Responsive on all devices | T097 | Device testing |

### Quality Gates (MUST PASS before proceeding)

1. **After Phase 2**: Design system validated, CSS variables work, no build errors
2. **After Phase 3**: Sidebar renders, navigation works, keyboard accessible
3. **After Phase 9**: Lighthouse ≥85, axe-core zero violations, all performance tests pass
4. **After Phase 10**: All 24 FRs and 15 SCs verified complete
5. **After Phase 12**: Production deployment successful, no critical errors in logs

---

## Task Checklist Tips

1. **Copy this file** into your project management tool (GitHub Issues, Linear, Jira, etc.)
2. **Mark as in progress** when starting a task
3. **Commit after each task** or logical group with clear commit message
4. **Stop at phase checkpoints** to validate independently
5. **Don't skip Phase 2** - it blocks all stories
6. **Parallel tasks** (marked [P]) can be assigned to different developers
7. **Tests are optional** but included for comprehensive validation
8. **Use file paths** as-is to ensure consistency across team

---

## Status

✅ **Task Generation Complete** - 133 tasks organized across 12 phases

- **Phase 1 (Setup)**: 5 tasks
- **Phase 2 (Foundational)**: 15 tasks
- **Phase 3 (US1)**: 11 tasks
- **Phase 4 (US2)**: 20 tasks
- **Phase 5 (US3)**: 7 tasks
- **Phase 6 (US4)**: 14 tasks
- **Phase 7 (US5)**: 5 tasks
- **Phase 8 (Theme Toggle)**: 7 tasks
- **Phase 9 (Performance & A11y)**: 16 tasks
- **Phase 10 (Documentation & QA)**: 11 tasks
- **Phase 11 (Polish)**: 8 tasks
- **Phase 12 (Deployment)**: 6 tasks

**Total**: 133 tasks, estimated 200-250 hours for full implementation with 1 developer, ~100 hours with 2 parallel developers

**MVP (User Story 1 + Theme Toggle only)**: ~40-50 tasks, ~2-3 weeks with 1-2 developers

Ready for implementation. Begin with Phase 1 and proceed sequentially, or parallelize Phases 3-7 with team capacity.
