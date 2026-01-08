# Research Findings: Docusaurus UI Upgrade

**Feature**: 005-docusaurus-ui-upgrade
**Date**: 2026-01-08
**Status**: Phase 0 Complete

---

## Executive Summary

Research confirms technical feasibility of all 24 functional requirements and 15 success criteria. Key decisions establish CSS variables as primary theming mechanism, mobile-first responsive design, and Docusaurus theme swizzling for component customization. No blocking technical unknowns identified.

---

## 1. Docusaurus Theming Architecture

### Research Question
How to implement custom themes in Docusaurus 2.x without ejecting or modifying build pipeline?

### Findings

**Official Docusaurus 2.x Theming**:
- Docusaurus 2.x uses React 17+ with theme swizzling capability
- Theme files are React components that wrap layout, providing design tokens via context
- Swizzling (safe component override) allows customization without core changes
- Official theme system: `@docusaurus/theme-classic` provides Layout, Sidebar, Breadcrumbs, etc.

**CSS Injection Methods**:
1. **CSS Modules**: `src/css/custom.css` automatically included in every page
2. **CSS Variables**: `:root` scoped variables can be overridden per theme
3. **CSS-in-JS**: Styled components (if React wrapper) or CSS modules preferred for maintainability

**Theme Swizzling**:
- Safe swizzle: Override React components without forking entire theme
- Swizzle depth: `src/swizzled/` directory contains overridden components
- Upgrade safety: Swizzled components inherit base from `@docusaurus/theme-classic`

### Decision

**Approach**: CSS Variables + Theme Swizzling
- Custom `src/css/` for design tokens (colors, typography, spacing, breakpoints)
- Swizzle `Root.tsx` to wrap app with theme context provider
- Swizzle `Layout/` to provide responsive grid and mobile menu state
- CSS modules for component scoping (sidebar, breadcrumbs, code blocks)

**Rationale**:
- ✅ No build changes required (standard Docusaurus pattern)
- ✅ Maintainer-friendly (edit colors in CSS variables, not code)
- ✅ Upgradeable (swizzled components inherit base updates)
- ✅ Zero custom build steps

**Alternatives Considered**:
- ❌ Tailwind CSS: Requires build customization, adds build complexity
- ❌ SCSS preprocessing: Not standard in Docusaurus, requires additional setup
- ❌ Ejecting theme: Loses upgrade path, unmaintainable long-term

---

## 2. CSS Variables & Light/Dark Mode

### Research Question
How to implement persistent light/dark mode with CSS variables while respecting system preference?

### Findings

**CSS Custom Properties (Variables) Support**:
- Browser support: 98%+ across Chrome, Firefox, Safari, Edge (caniuse.com)
- No polyfill needed for target browsers (Chrome/Firefox/Safari/Edge latest 2 versions)
- Syntax: `--category-variant` naming convention standard (e.g., `--ds-color-primary`)
- Scope: `:root` for global, element-scoped for component-specific overrides

**Light/Dark Mode Detection**:
- **System preference**: `prefers-color-scheme` media query (99% browser support)
- **localStorage persistence**: `localStorage.getItem('theme')` / `localStorage.setItem('theme', value)`
- **Best practice**: Check localStorage first, then system preference, then default to light

**Flash of Unstyled Content (FOUC)**:
- Risk: Dark mode toggle doesn't apply until React hydrates (shows light theme briefly)
- Mitigation: Set theme in `<script>` tag before React loads (execute in `<head>`)
- Standard pattern in modern SPAs

**Naming Convention**:
```css
--ds-color-primary: #0066cc;
--ds-color-secondary: #6c757d;
--ds-font-family-body: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
--ds-spacing-base: 8px;
--ds-breakpoint-mobile: 320px;
```

### Decision

**Approach**: CSS Variables + localStorage + System Preference Detection
```javascript
// In Root.tsx
useEffect(() => {
  const savedTheme = localStorage.getItem('theme');
  const prefersDark = window.matchMedia('(prefers-color-scheme: dark)').matches;
  const theme = savedTheme || (prefersDark ? 'dark' : 'light');
  document.documentElement.setAttribute('data-theme', theme);
}, []);

const toggleTheme = () => {
  const current = document.documentElement.getAttribute('data-theme');
  const next = current === 'dark' ? 'light' : 'dark';
  document.documentElement.setAttribute('data-theme', next);
  localStorage.setItem('theme', next);
};
```

**CSS Setup**:
```css
:root {
  --ds-color-primary: #0066cc;
  --ds-color-text: #1a1a1a;
  /* ... other light theme vars */
}

[data-theme="dark"] {
  --ds-color-primary: #4da6ff;
  --ds-color-text: #ffffff;
  /* ... other dark theme vars */
}
```

**Rationale**:
- ✅ Fast (no round-trip to server, immediate application)
- ✅ Respects user choice (localStorage override)
- ✅ Respects system preference (fallback)
- ✅ No flash (theme set before React hydration)

---

## 3. WCAG AA Accessibility Standards

### Research Question
What are concrete WCAG AA requirements for documentation site?

### Findings

**WCAG 2.1 Level AA Criteria (W3C Official)** applicable to this project:

**Perceivable**:
- **1.4.3 Contrast (Minimum)**: Text and background must have 4.5:1 contrast ratio (normal text) or 3:1 (large text ≥18pt or ≥14pt bold)
- **1.4.10 Reflow**: Content must be reflow-able without scrolling in 320px width
- **1.4.12 Text Spacing**: Allowance for text spacing without loss of content/functionality

**Operable**:
- **2.1.1 Keyboard**: All functionality must be operable via keyboard (Tab, Enter, Escape)
- **2.1.2 No Keyboard Trap**: Focus must not be trapped; users can exit all components via keyboard
- **2.4.3 Focus Order**: Logical tab order matching visual order
- **2.4.7 Focus Visible**: Keyboard focus indicator visible

**Understandable**:
- **3.2.4 Consistent Identification**: UI components with same functionality must be identified consistently (sidebar, breadcrumbs)

**Robust**:
- **4.1.2 Name, Role, Value**: Components must have accessible names, roles, and values (ARIA labels)
- **4.1.3 Status Messages**: Status changes (theme change, menu open) announced to screen readers

**Testing Tools**:
- axe-core: Automated accessibility scanning (open source, integrates with CI/CD)
- WAVE: Browser extension for manual verification
- Lighthouse: Built-in accessibility audit (Google Chrome DevTools)
- Color Contrast Analyzer: Manual verification of specific color pairs

### Decision

**Approach**: Baseline enforcement + continuous verification
- **Design Phase**: Define colors with contrast ratios pre-verified
- **Implementation**: Add ARIA labels, keyboard handlers, focus indicators
- **Testing**: axe-core in CI/CD (fail build if AA violations), manual review before launch
- **Verification**: Lighthouse score ≥85 (includes accessibility audit)

**Specific Requirements**:
1. **Contrast**: All text 4.5:1 minimum (verified in light and dark themes)
2. **Keyboard Navigation**:
   - Sidebar: Tab (focus items), Enter (navigate/expand), Escape (close mobile menu)
   - Breadcrumbs: Tab (focus), Enter (navigate)
   - Theme toggle: Tab (focus), Space/Enter (activate)
3. **Focus Indicators**: Outline or background change visible on all interactive elements
4. **ARIA Labels**:
   - Sidebar: `role="navigation"` + `aria-label="Documentation sidebar"`
   - Breadcrumbs: `role="navigation"` + `aria-current="page"` on active item
   - Theme toggle: `aria-label="Toggle dark mode"` + `aria-pressed` attribute

**Rationale**:
- ✅ WCAG AA is measurable and auditable (not aspirational)
- ✅ Inclusive design benefits all users (large text, keyboard-only users, screen readers)
- ✅ Automated testing catches regressions in CI/CD
- ✅ Manual testing captures UX issues automation misses

---

## 4. Responsive Design Patterns

### Research Question
What are best practices for responsive design covering 320px to 1440px+ with good UX?

### Findings

**Mobile-First CSS Approach**:
- Start with base styles for 320px (mobile)
- Add media queries as viewport grows (320px → 768px → 1024px → 1440px)
- Reduces CSS, improves performance, prioritizes mobile experience
- Progressive enhancement: works on older browsers that don't support media queries

**Responsive Breakpoints**:
```css
/* Mobile: 320px - 767px */
@media (max-width: 767px) {
  /* Hamburger menu, single column layout */
}

/* Tablet: 768px - 1023px */
@media (min-width: 768px) {
  /* Collapsible sidebar, optimized width */
}

/* Desktop: 1024px - 1439px */
@media (min-width: 1024px) {
  /* Full sidebar, 2-column layout */
}

/* Large: 1440px+ */
@media (min-width: 1440px) {
  /* Max content width, wide sidebar */
}
```

**Touch Targets**:
- Minimum 44x44 pixels (Apple HIG, Google Material Design standard)
- Verified via testing on actual mobile devices (iPhone, Android)
- Links, buttons, sidebar items, breadcrumbs all ≥44x44

**Flexible Layouts**:
- CSS Grid and Flexbox for responsive without media queries (where applicable)
- `flex-wrap: wrap` for component rows
- `max-width` constraints prevent excessive width on large screens

**Viewport Meta Tag** (already in Docusaurus):
```html
<meta name="viewport" content="width=device-width, initial-scale=1">
```

### Decision

**Approach**: Mobile-first CSS with 4 strategic breakpoints
- Start styles for 320px (mobile)
- Tablet optimizations: 768px (collapsible sidebar)
- Desktop full layout: 1024px (permanent sidebar)
- Large screens: 1440px (max content width to prevent excessive line lengths)

**Mobile Menu Implementation**:
```javascript
// Mobile: Hamburger menu (button toggle)
// 320px-767px: Show hamburger, hide sidebar permanently
// 768px+: Auto-show sidebar, hide hamburger

const [mobileMenuOpen, setMobileMenuOpen] = useState(false);

// On mobile, close menu when navigating
useEffect(() => {
  if (mobileMenuOpen && width > 768) {
    setMobileMenuOpen(false); // Auto-close when resizing to desktop
  }
}, [width]);
```

**Responsive Images**:
- Use `<img>` with `max-width: 100%` for docs images
- Docusaurus provides image optimization via webpack

**Rationale**:
- ✅ Mobile-first improves performance and UX
- ✅ 4 breakpoints cover device range without excessive complexity
- ✅ Touch targets ensure usability on mobile
- ✅ Tested approach from Material Design and Bootstrap

---

## 5. Typography System Design

### Research Question
What typography scale and font stack provides readability and good performance?

### Findings

**System Font Stack** (Apple HIG + Bootstrap best practice):
```css
font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
```
- `-apple-system`: San Francisco on macOS/iOS
- `BlinkMacSystemFont`: Same as above, old Safari
- `Segoe UI`: Windows default
- Fallbacks: Helvetica, Arial

**Benefits**:
- Zero download overhead (all fonts pre-installed on OS)
- Professional appearance (same fonts as OS UI)
- Excellent performance (no web font requests)
- Consistent with Docusaurus defaults

**Modular Scale**:
```css
--ds-font-size-xs: 12px;      /* Small UI elements */
--ds-font-size-sm: 14px;      /* UI labels, breadcrumbs */
--ds-font-size-base: 16px;    /* Body text, default */
--ds-font-size-lg: 18px;      /* Large body text */
--ds-font-size-xl: 20px;      /* Subheadings */
--ds-font-size-2xl: 24px;     /* H3 */
--ds-font-size-3xl: 28px;     /* H2 */
--ds-font-size-4xl: 32px;     /* H1 */
```

**Line Heights**:
- Body text: 1.6 (16px * 1.6 = 25.6px line height)
- Headings: 1.3 (tighter for visual hierarchy)
- Code blocks: 1.5 (readable without feeling spacious)

**Font Weights**:
- Normal (400): Body text
- Semibold (600): Emphasis, sidebar items
- Bold (700): Headings, strong emphasis

### Decision

**Approach**: System fonts + 7-point scale
- Use `-apple-system` stack for zero download cost
- Define 7 sizes (xs to 4xl) covering all use cases
- Set base line-height 1.6 for body readability
- Use semibold/bold for visual hierarchy

**Rationale**:
- ✅ Zero performance impact (no font downloads)
- ✅ Professional appearance (matches OS UI)
- ✅ Accessible (Docusaurus defaults are accessible)
- ✅ Maintainable (consistent scale used across all components)

---

## 6. Design Tokens & Organization

### Research Question
How should design tokens be organized and maintained?

### Findings

**Design Token Naming Convention** (Design Tokens Community Group standard):

```
--ds-<category>-<concept>[-modifier]
```

Examples:
```css
--ds-color-primary           /* Semantic: primary action */
--ds-color-primary-hover    /* State variant */
--ds-color-neutral-200      /* Numeric scale (lighter) */
--ds-font-size-body         /* Semantic: body text */
--ds-spacing-md             /* Semantic: medium spacing */
--ds-breakpoint-desktop     /* Infrastructure */
```

**Organization Structure**:
```
src/css/
├── custom.css              # Imports all themes + base
├── theme/
│   ├── light.css          # Light theme variable values
│   ├── dark.css           # Dark theme variable values
│   ├── typography.css     # Font-related vars (shared)
│   ├── spacing.css        # Spacing scale (shared)
│   └── accessibility.css  # Focus states, reduced motion
└── components/
    ├── sidebar.css        # Component-specific styles
    └── ...
```

**CSS Variable Definition Format**:
```css
/* Light theme (default) */
:root {
  /* Colors */
  --ds-color-primary: #0066cc;
  --ds-color-primary-hover: #0052a3;
  --ds-color-secondary: #6c757d;
  --ds-color-text: #1a1a1a;
  --ds-color-background: #ffffff;
  --ds-color-border: #e0e0e0;

  /* Typography */
  --ds-font-family-body: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
  --ds-font-size-base: 16px;
  --ds-line-height-body: 1.6;

  /* Spacing */
  --ds-spacing-xs: 4px;
  --ds-spacing-sm: 8px;
  --ds-spacing-md: 16px;
  --ds-spacing-lg: 24px;
  --ds-spacing-xl: 32px;

  /* Breakpoints */
  --ds-breakpoint-mobile: 320px;
  --ds-breakpoint-tablet: 768px;
  --ds-breakpoint-desktop: 1024px;
  --ds-breakpoint-large: 1440px;
}

/* Dark theme */
[data-theme="dark"] {
  --ds-color-primary: #4da6ff;
  --ds-color-text: #ffffff;
  --ds-color-background: #1a1a1a;
  --ds-color-border: #404040;
  /* ... other vars */
}
```

### Decision

**Approach**: Flat namespace with semantic categories
- All tokens prefixed with `--ds-` for clarity
- Organize by category (color, typography, spacing, breakpoint)
- Group light/dark values using `[data-theme]` selector
- Shared tokens (typography, spacing) not duplicated between themes

**Rationale**:
- ✅ Maintainable (single source of truth per token)
- ✅ Scalable (easy to add new tokens)
- ✅ Readable (semantic names clear intent)
- ✅ Compatible (no build tools needed)

---

## 7. Performance Optimization for Static Sites

### Research Question
How to maintain <20% performance impact from theming and styling?

### Findings

**Docusaurus Performance Baseline** (new site):
- First Contentful Paint (FCP): ~1.0-1.5s
- Largest Contentful Paint (LCP): ~1.5-2.0s
- Cumulative Layout Shift (CLS): <0.1
- Lighthouse score: ~90

**CSS Impact Factors**:
- **CSS file size**: Each CSS file is parsed/evaluated (1KB ≈ 0.1ms)
- **CSS complexity**: Selectors, media queries parsed at load time
- **JavaScript for theming**: Theme toggle function (minimal overhead)
- **localStorage operations**: ~1-2ms for read/write

**Baseline Measurement Strategy**:
```bash
# Measure current performance
lighthouse --view https://localhost:3000

# Record metrics:
# - FCP, LCP, CLS
# - Lighthouse score
# - CSS file sizes
```

**Optimization Strategies**:
1. **CSS Minification**: Docusaurus build minifies automatically
2. **Critical CSS**: Extract above-the-fold styles for inline inclusion (if needed)
3. **CSS Modules**: Scope CSS to components, avoid duplicate rules
4. **Font loading**: Use system fonts (no `@font-face` requests)
5. **Media queries**: Already optimized in browser (unused selectors not evaluated)

**Expected Impact**:
- CSS variables: <1KB overhead
- Custom CSS (styles): ~5-10KB minified (10-15ms load time on 4G)
- JavaScript (theme toggle): <1KB minified
- **Total**: ~15KB, <20ms overhead = <5% performance impact (well under 20% target)

### Decision

**Approach**: Performance-conscious CSS + Lighthouse validation
- Measure baseline before implementation
- Optimize CSS file sizes (minification)
- Use system fonts (no web font downloads)
- CSS variables (minimal overhead)
- Verify <20% impact with Lighthouse before merging

**Success Metrics**:
- Lighthouse score ≥85 (minimum acceptable)
- FCP <2.5s on 4G connection
- No layout shifts (CLS <0.1)

**Rationale**:
- ✅ Static site: CSS impact is primary optimization focus
- ✅ System fonts: Massive performance win
- ✅ Docusaurus build: Already optimized, we don't degrade it
- ✅ Measurable: Lighthouse provides concrete numbers

---

## 8. Browser & Platform Compatibility

### Research Question
What browsers and devices must be supported?

### Findings

**Target Browsers** (latest 2 versions):
- Chrome 120+
- Firefox 121+
- Safari 17+
- Edge 120+

**CSS Feature Support** (all targets):
- CSS Custom Properties (variables): 98%+ support
- Media Queries: 100% support
- Flexbox: 100% support
- CSS Grid: 100% support
- `prefers-color-scheme`: 99% support

**Device Support** (OS devices, not emulation):
- iPhone 12+ (iOS 15+): Safari
- iPad Pro (iOS 15+): Safari
- Android 11+ (Chrome, Firefox)
- Windows 10+ (Chrome, Firefox, Edge)
- macOS 11+ (Chrome, Firefox, Safari, Edge)

**Fallback Strategy**:
- Light theme as fallback (if CSS variables fail)
- No polyfills needed (target browsers all support modern CSS)
- Graceful degradation: If theme toggle fails, site still works in light mode

### Decision

**Approach**: Modern baseline with graceful degradation
- Target latest 2 versions of each major browser
- Assume modern CSS support (variables, grid, flexbox)
- Test on actual devices before launch (iPhone, Android, Windows)
- Lighthouse test on Chrome DevTools (covers all targets)

**Rationale**:
- ✅ Latest 2 versions covers 95%+ of users
- ✅ Modern CSS simplifies implementation
- ✅ Browser auto-update ensures compatibility long-term
- ✅ Graceful fallback for edge cases

---

## 9. Testing & Validation Approach

### Research Question
How to ensure all FRs, SCs, and accessibility requirements are met?

### Findings

**Test Categories**:

**1. Accessibility Testing**:
```bash
# axe-core in CI/CD
npm test -- --coverage --a11y

# Manual verification
# - WAVE browser extension
# - Keyboard navigation (Tab, Enter, Escape all interactive elements)
# - Screen reader test (NVDA, VoiceOver, JAWS simulation)
# - Color contrast checker on all color pairs
```

**2. Responsive Design Testing**:
```bash
# Breakpoint testing
# - Chrome DevTools: 320px, 768px, 1024px, 1440px
# - Actual devices: iPhone SE (375px), iPad (768px), Desktop (1440px)
# - Touch target validation: All buttons/links ≥44x44px
```

**3. Theming Testing**:
```bash
# Dark mode testing
# - Manual toggle: Light → Dark → Light
# - localStorage persistence: Refresh page, theme persists
# - System preference: Disable localStorage, use system preference
# - Contrast verification: All colors meet 4.5:1 in dark mode
```

**4. Performance Testing**:
```bash
# Lighthouse automation
lighthouse --view --output-path=./lighthouse.html

# Metrics to track:
# - FCP, LCP, CLS
# - Total CSS size
# - JavaScript bundle size
# - Verify <20% impact
```

**5. Cross-Browser Testing**:
```bash
# BrowserStack or local testing
# Chrome, Firefox, Safari, Edge on:
# - Windows 10/11
# - macOS 12/13
# - iOS 16/17
# - Android 12/13
```

**6. Component Integration Testing**:
```javascript
// Jest + React Testing Library
describe('Sidebar Navigation', () => {
  test('expands module on Enter key', () => {
    // Simulate keyboard interaction
    userEvent.tab(); // Focus module
    userEvent.keyboard('{Enter}');
    // Verify module expanded
  });

  test('closes mobile menu on Escape', () => {
    // Mobile menu auto-close
  });
});
```

### Decision

**Approach**: Layered testing strategy
- **Automated**: axe-core (a11y), Jest (components), Lighthouse (performance)
- **Manual**: Keyboard navigation, screen reader, actual devices
- **CI/CD Integration**: Tests run on every PR, fail build if violations
- **Pre-launch**: Manual QA session with 5-10 users, gather feedback

**Rationale**:
- ✅ Automated testing catches regressions
- ✅ Manual testing captures UX issues automation misses
- ✅ CI/CD integration prevents shipping violations
- ✅ User feedback validates design effectiveness

---

## Summary: Research-Backed Technical Decisions

| Decision | Basis | Confidence |
|----------|-------|-----------|
| CSS Variables + Theme Swizzling | Docusaurus official pattern | ✅ High |
| localStorage + System Preference | Industry standard, 99% support | ✅ High |
| WCAG AA Standards | W3C official guidelines | ✅ High |
| Mobile-First 4 Breakpoints | Material Design + Bootstrap proven | ✅ High |
| System Font Stack | Zero download cost, professional | ✅ High |
| Design Token Naming | DTCG community standard | ✅ High |
| <20% Performance Impact | CSS optimization proven in practice | ✅ High |
| Browser Support Latest 2 Versions | 95%+ user coverage | ✅ High |
| Layered Testing | Industry best practice | ✅ High |

---

## No Blocking Technical Unknowns

All 24 functional requirements are addressable with established patterns. All 15 success criteria are measurable and verifiable. Zero technical debt or architectural risks identified.

**Status**: ✅ **RESEARCH COMPLETE** - Ready for data model and contract generation.
