# Theme Customization Quickstart Guide

**Feature**: 005-docusaurus-ui-upgrade
**Date**: 2026-01-08
**Audience**: Site maintainers, documentation team

---

## Overview

This guide shows how to customize the Docusaurus documentation theme colors, fonts, and spacing without writing code or modifying content.

All customization is done via CSS variables in `src/css/custom.css`. No build steps required—changes are live on page refresh.

---

## Quick Customization Tasks

### 1. Change Primary Color

**Current**: Blue (#0066cc)
**Task**: Change to your organization's brand color

**Steps**:

1. Open `src/css/custom.css`
2. Find the `:root` section (top of file)
3. Locate this line:
   ```css
   --ds-color-primary: #0066cc;
   ```
4. Replace with your color (any valid hex code):
   ```css
   --ds-color-primary: #2563eb;  /* New blue */
   ```
5. Save file
6. Refresh browser—change takes effect immediately

**Verification**: Links, buttons, and highlights should reflect your new color.

### 2. Change Light/Dark Theme Colors Separately

**Task**: Make dark theme warmer (orange tint instead of cool blue)

**Steps**:

1. Open `src/css/custom.css`
2. Find the `[data-theme="dark"]` section (lower in file)
3. Locate:
   ```css
   --ds-color-primary: #4da6ff;
   --ds-color-accent: #ffb366;
   ```
4. Customize:
   ```css
   --ds-color-primary: #ff9966;   /* Warm orange */
   --ds-color-accent: #ffcc99;    /* Lighter orange */
   ```
5. Save and refresh

**Note**: Light theme colors are in `:root` (top section); dark theme overrides are in `[data-theme="dark"]`.

---

### 3. Change Font Family

**Current**: System fonts (San Francisco on macOS, Segoe UI on Windows)
**Task**: Use a custom web font like "Georgia" for headings

**Steps**:

1. Open `src/css/custom.css`
2. Find typography section:
   ```css
   --ds-font-family-body: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
   --ds-font-family-heading: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
   ```
3. Replace heading font:
   ```css
   --ds-font-family-heading: Georgia, serif;
   ```
4. Save and refresh

**Options**:
- System fonts (recommended, zero cost): `Arial`, `Georgia`, `Courier`, `Verdana`
- Web fonts (requires download): Import from Google Fonts (add instructions if implementing)
- Current recommendation: **Keep system fonts for best performance**

### 4. Change Text Size Scale

**Current**: 16px base, 4xl = 32px
**Task**: Make all text slightly larger (18px base, 4xl = 36px)

**Steps**:

1. Open `src/css/custom.css`
2. Find typography sizes:
   ```css
   --ds-font-size-base: 16px;
   --ds-font-size-4xl: 32px;
   --ds-font-size-3xl: 28px;
   ```
3. Increase by 2px each:
   ```css
   --ds-font-size-base: 18px;
   --ds-font-size-4xl: 36px;
   --ds-font-size-3xl: 30px;
   --ds-font-size-2xl: 26px;
   --ds-font-size-xl: 22px;
   --ds-font-size-lg: 20px;
   --ds-font-size-sm: 16px;
   --ds-font-size-xs: 14px;
   ```
4. Save and refresh

**Note**: Adjust all sizes proportionally to maintain visual hierarchy.

### 5. Change Spacing (Padding/Margins)

**Current**: 8px base unit
**Task**: Make components more spacious (12px base)

**Steps**:

1. Open `src/css/custom.css`
2. Find spacing scale:
   ```css
   --ds-spacing-base: 8px;
   --ds-spacing-xs: 4px;
   --ds-spacing-sm: 8px;
   --ds-spacing-md: 16px;
   --ds-spacing-lg: 24px;
   ```
3. Multiply all by 1.5:
   ```css
   --ds-spacing-base: 12px;
   --ds-spacing-xs: 6px;
   --ds-spacing-sm: 12px;
   --ds-spacing-md: 24px;
   --ds-spacing-lg: 36px;
   --ds-spacing-xl: 48px;
   --ds-spacing-2xl: 72px;
   ```
4. Save and refresh

**Effect**: All padding and margins increase, making layout less dense.

---

### 6. Change Contrast (Dark Mode Text Color)

**Task**: Make dark mode text easier to read (whiter white)

**Steps**:

1. Find dark theme text colors:
   ```css
   [data-theme="dark"] {
     --ds-color-text-primary: #ffffff;      /* Already white */
     --ds-color-text-secondary: #d0d0d0;    /* Light gray */
   ```
2. For more contrast:
   ```css
   --ds-color-text-secondary: #e0e0e0;      /* Lighter gray */
   ```
3. Save and refresh

**Verify**: Check contrast ratio with accessibility checker to ensure ≥4.5:1.

### 7. Change Breakpoint (Responsive Sizes)

**Current**: Mobile (320px), Tablet (768px), Desktop (1024px), Large (1440px)
**Task**: Adjust tablet breakpoint to 700px

**Steps**:

1. Find breakpoints:
   ```css
   --ds-breakpoint-mobile: 320px;
   --ds-breakpoint-tablet: 768px;
   --ds-breakpoint-desktop: 1024px;
   --ds-breakpoint-large: 1440px;
   ```
2. Edit:
   ```css
   --ds-breakpoint-tablet: 700px;
   ```
3. Save and refresh

**Note**: Changing breakpoints requires browser width testing. Test with Chrome DevTools at different widths.

---

## Complete Color Palette Reference

### Light Theme (Default)

```css
:root {
  --ds-color-primary: #0066cc;           /* Links, buttons */
  --ds-color-secondary: #6c757d;         /* Secondary text */
  --ds-color-accent: #ff6b35;            /* Highlights */
  --ds-color-success: #28a745;           /* Green */
  --ds-color-warning: #ffc107;           /* Yellow */
  --ds-color-error: #dc3545;             /* Red */
  --ds-color-text-primary: #1a1a1a;      /* Body text */
  --ds-color-background: #ffffff;        /* Page bg */
  --ds-color-border: #dee2e6;            /* Borders */
}
```

### Dark Theme

```css
[data-theme="dark"] {
  --ds-color-primary: #4da6ff;           /* Lighter blue */
  --ds-color-secondary: #adb5bd;         /* Lighter gray */
  --ds-color-accent: #ffb366;            /* Lighter orange */
  --ds-color-success: #52c41a;           /* Brighter green */
  --ds-color-warning: #faad14;           /* Brighter yellow */
  --ds-color-error: #ff7875;             /* Brighter red */
  --ds-color-text-primary: #ffffff;      /* White text */
  --ds-color-background: #1a1a1a;        /* Dark bg */
  --ds-color-border: #404040;            /* Dark borders */
}
```

---

## Testing Your Changes

After modifying CSS variables, verify:

1. **Visual Check**: Open site in browser, navigate pages, check colors apply
2. **Color Contrast**: Use [WebAIM Color Contrast Checker](https://webaim.org/resources/contrastchecker/) to verify 4.5:1 minimum
3. **Both Themes**: Test light theme and dark theme (toggle at top right)
4. **Multiple Pages**: Check module pages, different chapter types
5. **Responsive**: Test on mobile (Chrome DevTools: 375px), tablet (768px), desktop (1024px)

---

## Common Customization Scenarios

### Scenario 1: Brand Color + Accent
```css
/* Make everything match corporate purple */
:root {
  --ds-color-primary: #6366f1;        /* Purple */
  --ds-color-accent: #ec4899;         /* Pink accent */
}

[data-theme="dark"] {
  --ds-color-primary: #a78bfa;        /* Lighter purple */
  --ds-color-accent: #f472b6;         /* Lighter pink */
}
```

### Scenario 2: Higher Contrast Accessibility
```css
/* Increase contrast for visually impaired readers */
:root {
  --ds-color-text-primary: #000000;   /* Pure black (was #1a1a1a) */
  --ds-color-background: #ffffff;     /* Pure white (already) */
  --ds-color-secondary: #000000;      /* Black (was #6c757d) */
}
```

### Scenario 3: Relaxed/Spacious Layout
```css
/* Make site feel less cramped */
--ds-spacing-base: 10px;              /* Was 8px */
--ds-spacing-sm: 10px;
--ds-spacing-md: 20px;
--ds-spacing-lg: 30px;
--ds-spacing-xl: 40px;
--ds-line-height-relaxed: 1.8;        /* Was 1.6 */
```

### Scenario 4: Larger Default Text
```css
/* Help readers with vision challenges */
--ds-font-size-base: 18px;            /* Was 16px */
--ds-font-size-lg: 20px;
--ds-font-size-xl: 22px;
--ds-font-size-2xl: 26px;
--ds-font-size-3xl: 30px;
--ds-font-size-4xl: 36px;
--ds-line-height-relaxed: 1.8;
```

---

## CSS Variable Naming Convention

All tokens follow this pattern: `--ds-<category>-<variant>`

| Category | Examples | Notes |
|----------|----------|-------|
| **color** | `--ds-color-primary`, `--ds-color-text-primary` | Color values |
| **font-family** | `--ds-font-family-body`, `--ds-font-family-code` | Font stacks |
| **font-size** | `--ds-font-size-base`, `--ds-font-size-4xl` | Sizes in px |
| **font-weight** | `--ds-font-weight-bold` | 400, 600, 700 |
| **line-height** | `--ds-line-height-relaxed` | Unitless multipliers |
| **spacing** | `--ds-spacing-sm`, `--ds-spacing-lg` | Padding/margins |
| **breakpoint** | `--ds-breakpoint-tablet`, `--ds-breakpoint-large` | Responsive widths |

**Example**: To change body text color, edit `--ds-color-text-primary`

---

## Advanced: Creating a Custom Color Variant

**Task**: Add a new "highlight" background color for important boxes

**Steps**:

1. Add new variable to `:root`:
   ```css
   --ds-color-highlight-bg: #fff3cd;     /* Light yellow */
   ```

2. Add dark theme variant:
   ```css
   [data-theme="dark"] {
     --ds-color-highlight-bg: #664d00;   /* Dark brown */
   }
   ```

3. Use in component styles:
   ```css
   .highlight-box {
     background-color: var(--ds-color-highlight-bg);
     padding: var(--ds-spacing-md);
     border-radius: 4px;
   }
   ```

4. Verify contrast between text and new background color

---

## Troubleshooting

### Colors Don't Change After Edit
- **Cause**: Browser cache
- **Fix**: Hard refresh (Ctrl+Shift+R on Windows, Cmd+Shift+R on Mac)

### Spacing Looks Off After Changing Base Unit
- **Cause**: Not all spacing values scaled
- **Fix**: Update all spacing tokens proportionally (0, xs, sm, md, lg, xl, 2xl, 3xl, 4xl)

### Dark Mode Colors Look Bad
- **Cause**: Insufficient contrast between text and background
- **Fix**: Use [WebAIM Color Contrast Checker](https://webaim.org/resources/contrastchecker/) to verify 4.5:1 minimum

### Text Size Breaks Layout
- **Cause**: Increased font size causes wrapping
- **Fix**: Also increase line-height slightly (1.6 → 1.75) and test on mobile

### Responsive Breakpoints Not Working
- **Cause**: Media queries in component CSS not updated
- **Fix**: Search `@media` in CSS files and verify they use `--ds-breakpoint-*` variables

---

## Deployment

After customizing:

1. **Save** `src/css/custom.css`
2. **Test** locally (refresh browser, check all pages)
3. **Commit** change with message: "Customize theme: [description]"
4. **Push** to repository
5. **Deploy** (automatic via CI/CD)

No build steps required. CSS changes are live on next page load.

---

## Next Steps

- **More customization?** See [design-system.contract.md](contracts/design-system.contract.md) for complete token reference
- **Need colors verified?** Use [WebAIM Color Contrast Checker](https://webaim.org/resources/contrastchecker/)
- **Component styling?** See individual component CSS files in `src/css/components/`
- **Create new component?** Follow token usage rules in [design-system.contract.md](contracts/design-system.contract.md)

---

**Status**: ✅ Quickstart Guide Complete - Ready for implementation
