"use client";

import { useTheme } from "fumadocs-ui/provider/base";
import { Moon, Sun } from "lucide-react";

/**
 * Single-button light/dark toggle, in place of Fumadocs' three-way
 * `ThemeSwitch`. The provider still defaults to `system` on a first visit —
 * this only drops it as an explicit choice once someone picks a side.
 *
 * Which icon shows is decided by CSS (`dark:`) rather than by reading the
 * resolved theme during render: the theme is unknown on the server, so
 * rendering from it would mismatch on hydration and flash the wrong icon.
 */
export function ThemeToggle() {
  const { resolvedTheme, setTheme } = useTheme();

  return (
    <button
      type="button"
      aria-label="Toggle light and dark theme"
      onClick={() => setTheme(resolvedTheme === "dark" ? "light" : "dark")}
      // `-me-2` claws back half of the sidebar header's `p-4`, so the icon sits
      // closer to the edge than the logo does.
      className="-me-2 inline-flex size-8 cursor-pointer items-center justify-center text-fd-muted-foreground transition-colors hover:text-fd-accent-foreground"
    >
      <Sun className="size-4 dark:hidden" />
      <Moon className="hidden size-4 dark:block" />
    </button>
  );
}
